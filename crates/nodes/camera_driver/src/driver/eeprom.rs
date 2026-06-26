use std::fs::{File, OpenOptions};
use std::io;
use std::os::fd::AsRawFd;

use color_eyre::eyre::{Result, WrapErr, bail, ensure};

/// Fixed SC132GS EEPROM I2C address.
const EEPROM_I2C_ADDR: u16 = 0x50;
/// Offset of the SC132GS calibration validity flag.
const SC132GS_FLAG_OFFSET: u16 = 0x0000;
/// Offset of the packed SC132GS stereo calibration block.
const SC132GS_CALIB_OFFSET: u16 = 0x0022;
/// Size of the packed SC132GS stereo calibration block.
const SC132GS_CALIB_SIZE: usize = 39 * 8 + 4;
/// Linux I2C read-message flag.
const I2C_M_RD: u16 = 0x0001;
/// Linux combined I2C transfer ioctl number.
const I2C_RDWR: libc::c_ulong = 0x0707;

/// Linux `i2c_msg` layout used by `I2C_RDWR`.
#[repr(C)]
struct I2cMsg {
    /// 7-bit I2C device address.
    addr: u16,
    /// Transfer flags such as `I2C_M_RD`.
    flags: u16,
    /// Number of bytes in `buf`.
    len: u16,
    /// Mutable transfer buffer pointer.
    buf: *mut u8,
}

/// Linux `i2c_rdwr_ioctl_data` layout used by `I2C_RDWR`.
#[repr(C)]
struct I2cRdwrIoctlData {
    /// Pointer to an array of transfer messages.
    msgs: *mut I2cMsg,
    /// Number of transfer messages.
    nmsgs: u32,
}

/// Intrinsic and distortion calibration for one camera.
#[derive(Clone, Debug)]
pub struct CameraCalibration {
    /// Horizontal focal length in pixels.
    pub fx: f64,
    /// Vertical focal length in pixels.
    pub fy: f64,
    /// Horizontal principal point in pixels.
    pub cx: f64,
    /// Vertical principal point in pixels.
    pub cy: f64,
    /// Rational-polynomial distortion in OpenCV order.
    pub distortion: [f64; 8],
}

/// Complete stereo calibration parsed from the SC132GS EEPROM.
#[derive(Clone, Debug)]
pub struct StereoCalibration {
    /// Calibration image width.
    pub width: u32,
    /// Calibration image height.
    pub height: u32,
    /// Left camera intrinsics and distortion.
    pub left: CameraCalibration,
    /// Right camera intrinsics and distortion.
    pub right: CameraCalibration,
    /// Row-major rotation from left to right camera coordinates.
    pub rotation: [f64; 9],
    /// Translation from left to right camera coordinates in meters.
    pub translation: [f64; 3],
    /// Name of the distortion model represented by `distortion`.
    pub distortion_model: &'static str,
    /// Factory calibration rotation needed to map sensor orientation.
    pub cal_rotation_deg: u32,
}

impl StereoCalibration {
    /// Returns the absolute stereo baseline in meters.
    pub fn baseline_m(&self) -> f64 {
        self.translation[0].abs()
    }
}

/// Reads SC132GS stereo calibration from the selected camera I2C buses.
pub fn read_sc132gs_calibration<I>(buses: I) -> Result<StereoCalibration>
where
    I: IntoIterator<Item = u32>,
{
    let mut errors = Vec::new();
    for bus in buses {
        match read_sc132gs_calibration_on_bus(bus) {
            Ok(calib) => return Ok(calib),
            Err(err) => errors.push(format!("i2c-{bus}: {err:#}")),
        }
    }
    bail!(
        "SC132GS calibration EEPROM not found at 0x50 on selected camera buses ({})",
        errors.join("; ")
    )
}

/// Reads and parses SC132GS calibration from one I2C bus.
fn read_sc132gs_calibration_on_bus(bus: u32) -> Result<StereoCalibration> {
    let eeprom = Eeprom::open(bus)?;
    let mut flag = [0u8; 8];
    eeprom.read_at(SC132GS_FLAG_OFFSET, &mut flag)?;
    if flag[0] != 0x01 || !flag[1..].starts_with(b"SC") {
        bail!("unexpected EEPROM flag {:?}", flag);
    }

    let mut raw = [0u8; SC132GS_CALIB_SIZE];
    eeprom.read_at(SC132GS_CALIB_OFFSET, &mut raw)?;
    parse_sc132gs_calibration(&raw)
}

/// Parses and normalizes the packed SC132GS EEPROM calibration block.
fn parse_sc132gs_calibration(raw: &[u8; SC132GS_CALIB_SIZE]) -> Result<StereoCalibration> {
    let mut rd = F64Reader { raw, offset: 0 };

    let fxl = rd.next();
    let fyl = rd.next();
    let cxl = rd.next();
    let cyl = rd.next();
    let k1l = rd.next();
    let k2l = rd.next();
    let k3l = rd.next();
    let k4l = rd.next();
    let k5l = rd.next();
    let k6l = rd.next();
    let p1l = rd.next();
    let p2l = rd.next();
    let _rmsl = rd.next();

    let fxr = rd.next();
    let fyr = rd.next();
    let cxr = rd.next();
    let cyr = rd.next();
    let k1r = rd.next();
    let k2r = rd.next();
    let k3r = rd.next();
    let k4r = rd.next();
    let k5r = rd.next();
    let k6r = rd.next();
    let p1r = rd.next();
    let p2r = rd.next();
    let _rmsr = rd.next();

    let mut rotation = [0.0; 9];
    for value in &mut rotation {
        *value = rd.next();
    }
    let tx = rd.next();
    let ty = rd.next();
    let tz = rd.next();
    let _epilines = rd.next();

    let h_v = &raw[39 * 8..39 * 8 + 4];
    let cal_w = u16::from_be_bytes([h_v[0], h_v[1]]) as u32;
    let cal_h = u16::from_be_bytes([h_v[2], h_v[3]]) as u32;
    let width = if (101..10_000).contains(&cal_w) {
        cal_w
    } else {
        1280
    };
    let height = if (101..10_000).contains(&cal_h) {
        cal_h
    } else {
        1088
    };

    if !(100.0..=5000.0).contains(&fxl) || !(100.0..=5000.0).contains(&fxr) {
        bail!("invalid focal lengths fxl={fxl:.3} fxr={fxr:.3}");
    }

    let mut left = CameraCalibration {
        fx: fxl,
        fy: fyl,
        cx: cxl,
        cy: cyl,
        distortion: [k1l, k2l, p1l, p2l, k3l, k4l, k5l, k6l],
    };
    let mut right = CameraCalibration {
        fx: fxr,
        fy: fyr,
        cx: cxr,
        cy: cyr,
        distortion: [k1r, k2r, p1r, p2r, k3r, k4r, k5r, k6r],
    };

    let swapped_left_right = tx > 0.0;
    let translation = if swapped_left_right {
        std::mem::swap(&mut left, &mut right);
        rotation.swap(1, 3);
        rotation.swap(2, 6);
        rotation.swap(5, 7);
        [
            -(rotation[0] * tx + rotation[1] * ty + rotation[2] * tz),
            -(rotation[3] * tx + rotation[4] * ty + rotation[5] * tz),
            -(rotation[6] * tx + rotation[7] * ty + rotation[8] * tz),
        ]
    } else {
        [tx, ty, tz]
    };

    ensure!(
        translation.iter().all(|v| v.is_finite()) && translation[0] != 0.0,
        "invalid translation vector {translation:?}"
    );

    Ok(StereoCalibration {
        width,
        height,
        left,
        right,
        rotation,
        translation,
        distortion_model: "rational_polynomial",
        cal_rotation_deg: 90,
    })
}

/// Sequential reader for little-endian `f64` calibration values.
struct F64Reader<'a> {
    /// Raw EEPROM calibration block.
    raw: &'a [u8; SC132GS_CALIB_SIZE],
    /// Current byte offset into `raw`.
    offset: usize,
}

impl F64Reader<'_> {
    /// Reads the next little-endian `f64` value.
    fn next(&mut self) -> f64 {
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&self.raw[self.offset..self.offset + 8]);
        self.offset += 8;
        f64::from_le_bytes(bytes)
    }
}

/// Open EEPROM device for one camera I2C bus.
struct Eeprom {
    /// Linux I2C bus number.
    bus: u32,
    /// Open `/dev/i2c-*` file descriptor.
    file: File,
}

impl Eeprom {
    /// Opens `/dev/i2c-{bus}` for combined offset-write/read transfers.
    fn open(bus: u32) -> Result<Self> {
        let path = format!("/dev/i2c-{bus}");
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(&path)
            .wrap_err_with(|| format!("open {path}"))?;
        Ok(Self { bus, file })
    }

    /// Reads bytes from a 16-bit EEPROM offset without programming the EEPROM.
    fn read_at(&self, offset: u16, out: &mut [u8]) -> Result<()> {
        ensure!(
            out.len() <= u16::MAX as usize,
            "read too large: {} bytes",
            out.len()
        );

        let mut addr_buf = [(offset >> 8) as u8, (offset & 0xff) as u8];
        let mut msgs = [
            I2cMsg {
                addr: EEPROM_I2C_ADDR,
                flags: 0,
                len: addr_buf.len() as u16,
                buf: addr_buf.as_mut_ptr(),
            },
            I2cMsg {
                addr: EEPROM_I2C_ADDR,
                flags: I2C_M_RD,
                len: out.len() as u16,
                buf: out.as_mut_ptr(),
            },
        ];
        let mut data = I2cRdwrIoctlData {
            msgs: msgs.as_mut_ptr(),
            nmsgs: msgs.len() as u32,
        };

        let rc = unsafe { libc::ioctl(self.file.as_raw_fd(), I2C_RDWR, &mut data) };
        if rc < 0 {
            bail!(
                "read EEPROM bus={} offset=0x{offset:04x} len={}: {}",
                self.bus,
                out.len(),
                io::Error::last_os_error()
            );
        }
        Ok(())
    }
}
