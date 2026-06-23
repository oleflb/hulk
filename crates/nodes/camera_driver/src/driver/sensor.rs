use std::fs::{self, OpenOptions};
use std::io;
use std::os::fd::AsRawFd;
use std::path::{Path, PathBuf};
use std::thread;
use std::time::Duration;

/// SC132GS chip-id register address.
const SC132GS_CHIP_ID_REG: u16 = 0x3107;
/// Expected SC132GS chip-id value.
const SC132GS_CHIP_ID: u16 = 0x0132;
/// Candidate SC132GS I2C addresses used by X5 camera modules.
const SC132GS_I2C_ADDRS: [u16; 4] = [0x30, 0x31, 0x32, 0x33];
/// Required SC132GS sensor clock frequency.
const SC132GS_MCLK_FREQ: u32 = 24_000_000;
/// GPIO enable bitmask from the SC132GS sensor config.
const SC132GS_GPIO_ENABLE_BIT: u32 = 0x01;
/// GPIO active-level bitmask from the SC132GS sensor config.
const SC132GS_GPIO_LEVEL_BIT: u32 = 0x00;
/// Linux I2C read-message flag.
const I2C_M_RD: u16 = 0x0001;
/// Linux combined I2C transfer ioctl number.
const I2C_RDWR: libc::c_ulong = 0x0707;
/// Device-tree suffixes for X5 MIPI host nodes.
const MIPI_HOST_SUFFIXES: [&str; 4] = ["3d060000", "3d070000", "3d080000", "3d090000"];

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

/// Prepared sensor host properties needed by VIO setup.
#[derive(Clone, Debug)]
pub struct SensorHost {
    /// X5 MIPI host index.
    pub host: i32,
    /// Linux I2C bus for this camera module.
    pub i2c_bus: u32,
    /// MIPI RX PHY id consumed by VIN.
    pub mipi_rx_phy: i32,
    /// Detected SC132GS I2C address.
    pub i2c_addr: u16,
    /// Whether host MCLK sysfs programming is required.
    pub mclk_configured: bool,
}

/// Device-tree properties read from one `vcon@*` camera node.
#[derive(Clone, Debug)]
struct VconProperties {
    /// Device-tree status string.
    status: String,
    /// Linux I2C bus number.
    bus: u32,
    /// MIPI RX PHY list from device tree.
    rx_phy: Vec<i32>,
    /// Optional sensor reset/power GPIO list.
    gpio_oth: Vec<i32>,
}

/// Prepares one SC132GS host and verifies the sensor chip id.
pub fn prepare_sc132gs_host(host: i32) -> Result<SensorHost, String> {
    let host_index = usize::try_from(host)
        .ok()
        .filter(|index| *index < MIPI_HOST_SUFFIXES.len())
        .ok_or_else(|| format!("invalid MIPI host {host}; expected 0..3"))?;

    ensure_mipi_host_free(host)?;
    let mclk_configured = mipi_mclk_is_configured(host_index)?;
    let vcon = read_vcon(host)?;
    if !dt_status_is_ok(&vcon.status) {
        return Err(format!(
            "vcon@{host} status is {:?}, expected okay",
            vcon.status
        ));
    }
    if vcon.rx_phy.len() < 2 {
        return Err(format!(
            "vcon@{host} rx_phy has {} entries, expected at least 2",
            vcon.rx_phy.len()
        ));
    }

    if mclk_configured {
        write_sysfs(
            &format!("/sys/class/vps/mipi_host{host}/param/snrclk_freq"),
            &SC132GS_MCLK_FREQ.to_string(),
        )?;
        write_sysfs(
            &format!("/sys/class/vps/mipi_host{host}/param/snrclk_en"),
            "1",
        )?;
    }

    pulse_sc132gs_gpios(host, &vcon.gpio_oth)?;
    let probe = probe_sc132gs(vcon.bus)?;

    Ok(SensorHost {
        host,
        i2c_bus: vcon.bus,
        mipi_rx_phy: vcon.rx_phy[1],
        i2c_addr: probe.i2c_addr,
        mclk_configured,
    })
}

/// Rejects hosts that are already configured by another VIO pipeline.
fn ensure_mipi_host_free(host: i32) -> Result<(), String> {
    let path = format!("/sys/class/vps/mipi_host{host}/status/cfg");
    let cfg = fs::read_to_string(&path).map_err(|err| format!("read {path}: {err}"))?;
    let first_line = cfg.lines().next().unwrap_or("").trim();
    if first_line != "not inited" {
        return Err(format!(
            "mipi_host{host} is already configured: status/cfg first line is {first_line:?}"
        ));
    }
    Ok(())
}

/// Checks whether the host has pinctrl-backed MCLK configuration.
fn mipi_mclk_is_configured(host_index: usize) -> Result<bool, String> {
    let suffix = MIPI_HOST_SUFFIXES[host_index];
    let path = PathBuf::from(format!("/proc/device-tree/soc/cam/mipi_host@{suffix}"));
    if !path.is_dir() {
        return Err(format!("missing device-tree node {}", path.display()));
    }
    let pinctrl_names = read_dt_string_optional(&path.join("pinctrl-names"))?;
    Ok(pinctrl_names
        .as_deref()
        .is_some_and(|value| !value.is_empty()))
}

/// Reads vcon properties for one MIPI host from device tree.
fn read_vcon(host: i32) -> Result<VconProperties, String> {
    let path = PathBuf::from(format!("/proc/device-tree/soc/cam/vcon@{host}"));
    if !path.is_dir() {
        return Err(format!("missing device-tree node {}", path.display()));
    }

    let status = read_dt_string(&path.join("status"))?;
    let bus = read_be_i32_scalar(&path.join("bus"))?;
    if bus < 0 {
        return Err(format!("vcon@{host} bus is negative: {bus}"));
    }
    let rx_phy = read_be_i32_array(&path.join("rx_phy"))?;
    let gpio_oth = read_be_i32_array_optional(&path.join("gpio_oth"))?;

    Ok(VconProperties {
        status,
        bus: bus as u32,
        rx_phy,
        gpio_oth,
    })
}

/// Returns true for accepted device-tree enabled status values.
fn dt_status_is_ok(status: &str) -> bool {
    status == "okay" || status == "ok"
}

/// Reads a required NUL-terminated device-tree string.
fn read_dt_string(path: &Path) -> Result<String, String> {
    read_dt_string_optional(path)?.ok_or_else(|| format!("missing {}", path.display()))
}

/// Reads an optional NUL-terminated device-tree string.
fn read_dt_string_optional(path: &Path) -> Result<Option<String>, String> {
    let raw = match fs::read(path) {
        Ok(raw) => raw,
        Err(err) if err.kind() == io::ErrorKind::NotFound => return Ok(None),
        Err(err) => return Err(format!("read {}: {err}", path.display())),
    };
    let end = raw.iter().position(|byte| *byte == 0).unwrap_or(raw.len());
    let value = std::str::from_utf8(&raw[..end])
        .map_err(|err| format!("{} is not UTF-8: {err}", path.display()))?
        .trim()
        .to_string();
    Ok(Some(value))
}

/// Reads the first big-endian `i32` from a device-tree property.
fn read_be_i32_scalar(path: &Path) -> Result<i32, String> {
    read_be_i32_array(path)?
        .into_iter()
        .next()
        .ok_or_else(|| format!("{} is empty", path.display()))
}

/// Reads all big-endian `i32` values from a required property.
fn read_be_i32_array(path: &Path) -> Result<Vec<i32>, String> {
    let raw = fs::read(path).map_err(|err| format!("read {}: {err}", path.display()))?;
    parse_be_i32_array(path, &raw)
}

/// Reads all big-endian `i32` values from an optional property.
fn read_be_i32_array_optional(path: &Path) -> Result<Vec<i32>, String> {
    let raw = match fs::read(path) {
        Ok(raw) => raw,
        Err(err) if err.kind() == io::ErrorKind::NotFound => return Ok(Vec::new()),
        Err(err) => return Err(format!("read {}: {err}", path.display())),
    };
    parse_be_i32_array(path, &raw)
}

/// Parses a big-endian `i32` array from raw device-tree bytes.
fn parse_be_i32_array(path: &Path, raw: &[u8]) -> Result<Vec<i32>, String> {
    if !raw.len().is_multiple_of(4) {
        return Err(format!(
            "{} length {} is not a multiple of 4",
            path.display(),
            raw.len()
        ));
    }
    Ok(raw
        .chunks_exact(4)
        .map(|chunk| i32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]))
        .collect())
}

/// Writes a sysfs value with path context on error.
fn write_sysfs(path: &str, value: &str) -> Result<(), String> {
    fs::write(path, value).map_err(|err| format!("write {path}: {err}"))
}

/// Pulses configured SC132GS GPIOs for reset/power sequencing.
fn pulse_sc132gs_gpios(host: i32, gpio_oth: &[i32]) -> Result<(), String> {
    let active = 1 - SC132GS_GPIO_LEVEL_BIT as i32;
    for (index, gpio) in gpio_oth.iter().copied().enumerate().take(8) {
        if gpio == 0 || (SC132GS_GPIO_ENABLE_BIT & (1 << index)) == 0 {
            continue;
        }
        pulse_gpio(gpio, active)
            .map_err(|err| format!("vcon@{host} gpio_oth[{index}]={gpio}: {err}"))?;
    }
    Ok(())
}

/// Exports and pulses one GPIO, then unexports it if this function exported it.
fn pulse_gpio(gpio: i32, active: i32) -> Result<(), String> {
    let gpio_dir = PathBuf::from(format!("/sys/class/gpio/gpio{gpio}"));
    let exported_here = export_gpio(gpio, &gpio_dir)?;
    wait_for_gpio_path(&gpio_dir.join("direction"))?;
    fs::write(gpio_dir.join("direction"), "out").map_err(|err| format!("set direction: {err}"))?;

    set_gpio_value(&gpio_dir, active)?;
    thread::sleep(Duration::from_millis(30));
    set_gpio_value(&gpio_dir, 1 - active)?;
    thread::sleep(Duration::from_millis(30));
    set_gpio_value(&gpio_dir, active)?;
    thread::sleep(Duration::from_millis(30));

    if exported_here {
        fs::write("/sys/class/gpio/unexport", gpio.to_string())
            .map_err(|err| format!("unexport gpio: {err}"))?;
    }
    Ok(())
}

/// Exports a GPIO unless it is already exported.
fn export_gpio(gpio: i32, gpio_dir: &Path) -> Result<bool, String> {
    if gpio_dir.is_dir() {
        return Ok(false);
    }
    match fs::write("/sys/class/gpio/export", gpio.to_string()) {
        Ok(()) => Ok(true),
        Err(_) if gpio_dir.is_dir() => Ok(false),
        Err(err) => Err(format!("export gpio: {err}")),
    }
}

/// Waits briefly for a sysfs GPIO path to appear.
fn wait_for_gpio_path(path: &Path) -> Result<(), String> {
    for _ in 0..100 {
        if path.exists() {
            return Ok(());
        }
        thread::sleep(Duration::from_millis(1));
    }
    Err(format!("{} did not appear within 100ms", path.display()))
}

/// Sets one exported GPIO value.
fn set_gpio_value(gpio_dir: &Path, value: i32) -> Result<(), String> {
    fs::write(gpio_dir.join("value"), value.to_string())
        .map_err(|err| format!("set value {value}: {err}"))
}

/// Result of probing one SC132GS sensor address.
#[derive(Copy, Clone, Debug)]
struct SensorProbe {
    /// Detected sensor I2C address.
    i2c_addr: u16,
}

/// Finds an SC132GS sensor on the selected camera I2C bus.
fn probe_sc132gs(bus: u32) -> Result<SensorProbe, String> {
    let mut errors = Vec::new();
    for addr in SC132GS_I2C_ADDRS {
        match i2c_read_reg16_data16(bus, addr, SC132GS_CHIP_ID_REG) {
            Ok(chip_id) if chip_id == SC132GS_CHIP_ID => return Ok(SensorProbe { i2c_addr: addr }),
            Ok(chip_id) => errors.push(format!(
                "addr=0x{addr:02x} chip_id=0x{chip_id:04x}, expected 0x{SC132GS_CHIP_ID:04x}"
            )),
            Err(err) => errors.push(format!("addr=0x{addr:02x}: {err}")),
        }
    }
    Err(format!(
        "SC132GS not found on i2c-{bus} reg=0x{SC132GS_CHIP_ID_REG:04x}: {}",
        errors.join("; ")
    ))
}

/// Reads a 16-bit big-endian register value via Linux `I2C_RDWR`.
fn i2c_read_reg16_data16(bus: u32, addr: u16, reg: u16) -> Result<u16, String> {
    let path = format!("/dev/i2c-{bus}");
    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .open(&path)
        .map_err(|err| format!("open {path}: {err}"))?;

    let mut reg_buf = [(reg >> 8) as u8, (reg & 0xff) as u8];
    let mut read_buf = [0u8; 2];
    let mut msgs = [
        I2cMsg {
            addr,
            flags: 0,
            len: reg_buf.len() as u16,
            buf: reg_buf.as_mut_ptr(),
        },
        I2cMsg {
            addr,
            flags: I2C_M_RD,
            len: read_buf.len() as u16,
            buf: read_buf.as_mut_ptr(),
        },
    ];
    let mut data = I2cRdwrIoctlData {
        msgs: msgs.as_mut_ptr(),
        nmsgs: msgs.len() as u32,
    };

    let rc = unsafe { libc::ioctl(file.as_raw_fd(), I2C_RDWR, &mut data) };
    if rc < 0 {
        return Err(format!(
            "read reg 0x{reg:04x}: {}",
            io::Error::last_os_error()
        ));
    }
    Ok(u16::from_be_bytes(read_buf))
}
