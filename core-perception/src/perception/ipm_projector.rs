use opencv::{
    prelude::*,
    core::{self, Mat, Size, Scalar},
    imgproc,
};

pub struct IpmProjector {
    transform_matrix: Mat,
    bev_size: i32,
}

impl IpmProjector {
    pub fn new(src_w: i32, src_h: i32, bev_size: i32) -> Result<Self, String> {
        let mut src_pts = core::Vector::<core::Point2f>::new();
        src_pts.push(core::Point2f::new(0.0, src_h as f32 - 1.0));
        src_pts.push(core::Point2f::new(src_w as f32 - 1.0, src_h as f32 - 1.0));
        src_pts.push(core::Point2f::new(src_w as f32 * 0.35, src_h as f32 * 0.55));
        src_pts.push(core::Point2f::new(src_w as f32 * 0.65, src_h as f32 * 0.55));

        let mut dst_pts = core::Vector::<core::Point2f>::new();
        dst_pts.push(core::Point2f::new(bev_size as f32 * 0.15, bev_size as f32 - 1.0));
        dst_pts.push(core::Point2f::new(bev_size as f32 * 0.85, bev_size as f32 - 1.0));
        dst_pts.push(core::Point2f::new(bev_size as f32 * 0.15, 0.0));
        dst_pts.push(core::Point2f::new(bev_size as f32 * 0.85, 0.0));

        let transform_matrix = imgproc::get_perspective_transform(&src_pts, &dst_pts, core::DECOMP_LU)
            .map_err(|e| e.to_string())?;

        Ok(Self {
            transform_matrix,
            bev_size,
        })
    }

    pub fn project(&self, class_map: &Mat) -> Result<Mat, String> {
        let mut binary_mask = Mat::new_rows_cols_with_default(
            class_map.rows(),
            class_map.cols(),
            core::CV_8UC1,
            Scalar::all(0.0)
        ).map_err(|e| e.to_string())?;

        for y in 0..class_map.rows() {
            let src_row = class_map.ptr(y).map_err(|e| e.to_string())?;
            let dst_row = binary_mask.ptr_mut(y).map_err(|e| e.to_string())?;
            
            let src_slice = unsafe { std::slice::from_raw_parts(src_row, class_map.cols() as usize) };
            let dst_slice = unsafe { std::slice::from_raw_parts_mut(dst_row, class_map.cols() as usize) };
            
            for x in 0..class_map.cols() as usize {
                if src_slice[x] == 0 {
                    dst_slice[x] = 0;   // Road class (0) is safe free space
                } else {
                    dst_slice[x] = 255; // Other classes are marked as obstacles
                }
            }
        }

        let mut bev_grid = Mat::default();
        imgproc::warp_perspective(
            &binary_mask,
            &mut bev_grid,
            &self.transform_matrix,
            Size::new(self.bev_size, self.bev_size),
            imgproc::INTER_NEAREST,
            core::BORDER_CONSTANT,
            Scalar::all(255.0) // Fill outside boundaries as obstacles for absolute safety
        ).map_err(|e| e.to_string())?;

        Ok(bev_grid)
    }
}
