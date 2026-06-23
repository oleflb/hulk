use super::eeprom::{CameraCalibration, StereoCalibration};

/// Fixed-size 3x3 matrix used for rectification math.
type Matrix3 = [[f64; 3]; 3];

/// Shared rectified camera intrinsics used by both output views.
#[derive(Copy, Clone, Debug)]
struct RectifiedIntrinsics {
    /// Horizontal focal length in pixels.
    fx: f64,
    /// Vertical focal length in pixels.
    fy: f64,
    /// Horizontal principal point in pixels.
    cx: f64,
    /// Vertical principal point in pixels.
    cy: f64,
}

/// Generates left and right hardware GDC bins from EEPROM stereo calibration.
pub fn generate_rectified_gdc_bins(
    calib: &StereoCalibration,
    raw_width: u32,
    raw_height: u32,
    out_width: u32,
    out_height: u32,
) -> Result<(GdcBin, GdcBin), String> {
    if calib.cal_rotation_deg != 90 {
        return Err(format!(
            "only SC132GS 90 degree calibration rotation is supported, got {}",
            calib.cal_rotation_deg
        ));
    }
    let landscape_width = raw_height;
    let landscape_height = raw_width;
    if out_width != landscape_width || out_height != landscape_height {
        return Err(format!(
            "strict GDC map currently requires full rotated output {}x{}, got {}x{}",
            landscape_width, landscape_height, out_width, out_height
        ));
    }

    let sx = landscape_width as f64 / calib.width as f64;
    let sy = landscape_height as f64 / calib.height as f64;
    let left_k = scale_camera(&calib.left, sx, sy);
    let right_k = scale_camera(&calib.right, sx, sy);
    let rect_k = RectifiedIntrinsics {
        fx: (left_k.fx + right_k.fx) * 0.5,
        fy: (left_k.fy + right_k.fy) * 0.5,
        cx: (left_k.cx + right_k.cx) * 0.5,
        cy: (left_k.cy + right_k.cy) * 0.5,
    };

    let (left_from_rect, right_from_rect) = rectification_rotations(calib)?;
    let mut left_points = generate_camera_map(
        &left_k,
        left_from_rect,
        rect_k,
        raw_width,
        raw_height,
        out_width,
        out_height,
    );
    let left = generate_gdc_bin(
        &mut left_points,
        raw_width,
        raw_height,
        out_width,
        out_height,
    )?;
    drop(left_points);

    let mut right_points = generate_camera_map(
        &right_k,
        right_from_rect,
        rect_k,
        raw_width,
        raw_height,
        out_width,
        out_height,
    );
    let right = generate_gdc_bin(
        &mut right_points,
        raw_width,
        raw_height,
        out_width,
        out_height,
    )?;

    Ok((left, right))
}

/// Scales camera intrinsics from calibration size to target landscape size.
fn scale_camera(cam: &CameraCalibration, sx: f64, sy: f64) -> CameraCalibration {
    CameraCalibration {
        fx: cam.fx * sx,
        fy: cam.fy * sy,
        cx: cam.cx * sx,
        cy: cam.cy * sy,
        distortion: cam.distortion,
    }
}

/// Computes camera-from-rectified rotations for left and right cameras.
fn rectification_rotations(calib: &StereoCalibration) -> Result<(Matrix3, Matrix3), String> {
    let r = mat_from_row_major(calib.rotation);
    let t = calib.translation;
    let x_axis = normalize([-t[0], -t[1], -t[2]])?;
    let right_forward_in_left = mat_transpose_vec(r, [0.0, 0.0, 1.0]);
    let avg_forward = normalize([
        right_forward_in_left[0],
        right_forward_in_left[1],
        1.0 + right_forward_in_left[2],
    ])
    .unwrap_or([0.0, 0.0, 1.0]);
    let mut y_axis = normalize(cross(avg_forward, x_axis)).unwrap_or([0.0, 1.0, 0.0]);
    if dot(y_axis, [0.0, 1.0, 0.0]) < 0.0 {
        y_axis = [-y_axis[0], -y_axis[1], -y_axis[2]];
    }
    let z_axis = normalize(cross(x_axis, y_axis))?;
    let rect_from_left = [x_axis, y_axis, z_axis];
    let left_from_rect = transpose(rect_from_left);
    let right_from_rect = mat_mul(r, left_from_rect);
    Ok((left_from_rect, right_from_rect))
}

/// Builds a per-pixel GDC source map for one rectified camera view.
fn generate_camera_map(
    cam: &CameraCalibration,
    cam_from_rect: Matrix3,
    rect_k: RectifiedIntrinsics,
    raw_width: u32,
    raw_height: u32,
    out_width: u32,
    out_height: u32,
) -> Vec<super::ffi::point_t> {
    let mut points = Vec::with_capacity(out_width as usize * out_height as usize);
    for y in 0..out_height {
        for x in 0..out_width {
            let xr = (x as f64 - rect_k.cx) / rect_k.fx;
            let yr = (y as f64 - rect_k.cy) / rect_k.fy;
            let src_ray = mat_vec(cam_from_rect, [xr, yr, 1.0]);
            let xn = src_ray[0] / src_ray[2];
            let yn = src_ray[1] / src_ray[2];
            let (xd, yd) = distort_rational(xn, yn, cam.distortion);
            let landscape_x = cam.fx * xd + cam.cx;
            let landscape_y = cam.fy * yd + cam.cy;

            let portrait_x = landscape_y.clamp(0.0, raw_width.saturating_sub(1) as f64);
            let portrait_y = (raw_height.saturating_sub(1) as f64 - landscape_x)
                .clamp(0.0, raw_height.saturating_sub(1) as f64);
            points.push(super::ffi::point_t {
                x: portrait_x,
                y: portrait_y,
            });
        }
    }
    points
}

/// Applies OpenCV rational-polynomial distortion to normalized coordinates.
fn distort_rational(x: f64, y: f64, d: [f64; 8]) -> (f64, f64) {
    let r2 = x * x + y * y;
    let r4 = r2 * r2;
    let r6 = r4 * r2;
    let radial_num = 1.0 + d[0] * r2 + d[1] * r4 + d[4] * r6;
    let radial_den = 1.0 + d[5] * r2 + d[6] * r4 + d[7] * r6;
    let radial = if radial_den.abs() > f64::EPSILON {
        radial_num / radial_den
    } else {
        radial_num
    };
    let xy2 = 2.0 * x * y;
    let x_dist = x * radial + d[2] * xy2 + d[3] * (r2 + 2.0 * x * x);
    let y_dist = y * radial + d[2] * (r2 + 2.0 * y * y) + d[3] * xy2;
    (x_dist, y_dist)
}

/// Converts a row-major array into a 3x3 matrix.
fn mat_from_row_major(v: [f64; 9]) -> Matrix3 {
    [[v[0], v[1], v[2]], [v[3], v[4], v[5]], [v[6], v[7], v[8]]]
}

/// Multiplies a 3x3 matrix by a 3-vector.
fn mat_vec(m: Matrix3, v: [f64; 3]) -> [f64; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

/// Multiplies the transpose of a 3x3 matrix by a 3-vector.
fn mat_transpose_vec(m: Matrix3, v: [f64; 3]) -> [f64; 3] {
    [
        m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
        m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
        m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2],
    ]
}

/// Multiplies two 3x3 matrices.
fn mat_mul(a: Matrix3, b: Matrix3) -> Matrix3 {
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    out
}

/// Transposes a 3x3 matrix.
fn transpose(m: Matrix3) -> Matrix3 {
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}

/// Computes the 3D cross product.
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Computes the 3D dot product.
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Normalizes a finite nonzero 3D vector.
fn normalize(v: [f64; 3]) -> Result<[f64; 3], String> {
    let n = dot(v, v).sqrt();
    if n <= f64::EPSILON || !n.is_finite() {
        return Err(format!("cannot normalize vector {v:?}"));
    }
    Ok([v[0] / n, v[1] / n, v[2] / n])
}

/// hbmem-backed GDC binary buffer passed to the hardware GDC node.
#[cfg(x5cam_x5_target)]
pub struct GdcBin {
    /// Common hbmem buffer containing the generated GDC binary.
    pub buf: super::ffi::hb_mem_common_buf_t,
}

impl Drop for GdcBin {
    /// Frees the hbmem buffer allocated for the GDC binary.
    fn drop(&mut self) {
        if self.buf.fd >= 0 {
            unsafe {
                super::ffi::hb_mem_free_buf(self.buf.fd);
            }
            self.buf.fd = -1;
        }
    }
}

/// Calls the SDK GDC generator and copies the result into hbmem.
fn generate_gdc_bin(
    points: &mut [super::ffi::point_t],
    raw_width: u32,
    raw_height: u32,
    out_width: u32,
    out_height: u32,
) -> Result<GdcBin, String> {
    use std::{mem, ptr};

    if points.len() != out_width as usize * out_height as usize {
        return Err(format!(
            "GDC point count mismatch: got {}, expected {}",
            points.len(),
            out_width as usize * out_height as usize
        ));
    }

    unsafe {
        let mut param: super::ffi::param_t = mem::zeroed();
        param.format = super::ffi::frame_format_FMT_SEMIPLANAR_420;
        param.in_.w = raw_width;
        param.in_.h = raw_height;
        param.out.w = out_width;
        param.out.h = out_height;
        param.diameter = raw_height as i32;
        param.fov = 180.0;

        let mut window: super::ffi::window_t = mem::zeroed();
        window.out_r.x = 0;
        window.out_r.y = 0;
        window.out_r.w = out_width as i32;
        window.out_r.h = out_height as i32;
        window.transform = super::ffi::gdc_transformation_CUSTOM;
        window.input_roi_r.x = 0;
        window.input_roi_r.y = 0;
        window.input_roi_r.w = raw_width as i32;
        window.input_roi_r.h = raw_height as i32;
        window.strength = 1.0;
        window.strengthY = 1.0;
        window.zoom = 1.0;
        window.keep_ratio = 1;
        window.FOV_h = 90.0;
        window.FOV_w = 90.0;
        window.trapezoid_left_angle = 90.0;
        window.trapezoid_right_angle = 90.0;
        window.custom.full_tile_calc = 1;
        window.custom.tile_incr_x = 50;
        window.custom.tile_incr_y = 50;
        window.custom.w = out_width as i32 - 1;
        window.custom.h = out_height as i32 - 1;
        window.custom.centerx = out_width as f64 / 2.0 - 1.0;
        window.custom.centery = out_height as f64 / 2.0 - 1.0;
        window.custom.points = points.as_mut_ptr();

        let mut raw_buf: *mut u32 = ptr::null_mut();
        let mut raw_size = 0u64;
        let ret = super::ffi::hbn_gen_gdc_bin(&param, &window, 1, &mut raw_buf, &mut raw_size);
        if ret != 0 || raw_buf.is_null() || raw_size == 0 {
            return Err(format!("hbn_gen_gdc_bin failed ret={ret} size={raw_size}"));
        }

        let mut bin_buf: super::ffi::hb_mem_common_buf_t = mem::zeroed();
        let flags = super::ffi::mem_usage_t_HB_MEM_USAGE_MAP_INITIALIZED
            | super::ffi::mem_usage_t_HB_MEM_USAGE_PRIV_HEAP_2_RESERVED
            | super::ffi::mem_usage_t_HB_MEM_USAGE_CPU_READ_OFTEN
            | super::ffi::mem_usage_t_HB_MEM_USAGE_CPU_WRITE_OFTEN
            | super::ffi::mem_usage_t_HB_MEM_USAGE_CACHED;
        let ret = super::ffi::hb_mem_alloc_com_buf(raw_size, flags as i64, &mut bin_buf);
        if ret != 0 || bin_buf.virt_addr.is_null() {
            super::ffi::hbn_free_gdc_bin(raw_buf);
            return Err(format!("hb_mem_alloc_com_buf for GDC bin failed ret={ret}"));
        }

        ptr::copy_nonoverlapping(raw_buf.cast::<u8>(), bin_buf.virt_addr, raw_size as usize);
        super::ffi::hbn_free_gdc_bin(raw_buf);
        let ret = super::ffi::hb_mem_flush_buf(bin_buf.fd, 0, raw_size);
        if ret != 0 {
            super::ffi::hb_mem_free_buf(bin_buf.fd);
            return Err(format!("hb_mem_flush_buf for GDC bin failed ret={ret}"));
        }

        Ok(GdcBin { buf: bin_buf })
    }
}
