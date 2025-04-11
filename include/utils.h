#ifndef UTILS_H
#define UTILS_H

#include <opencv2/opencv.hpp>

typedef struct _pinhole_t
{
    float px;
    float py;
    float fx;
    float fy;

    int width;
    int height;
} pinhole_t;

typedef struct _coordinate_t
{
    int x;
    int y;
    float weight[4];
} coordinate_t;

typedef enum
{
    INTERPOLATION_NEARESTNEIGHBOR, /**< Nearest neighbor interpolation */
    INTERPOLATION_BILINEAR,        /**< Bilinear interpolation */
    INTERPOLATION_BILINEAR_DEPTH   /**< Bilinear interpolation with invalidation when neighbor contain invalid
                                                 data with value 0 */
} interpolation_t;

#define WRITE_FILES true
#define DO_GAMMA_CORRECT true



void create_undistortion_lut(const k4a_calibration_t *calibration,
                                    const k4a_calibration_type_t camera,
                                    const pinhole_t *pinhole,
                                    k4a_image_t lut,
                                    interpolation_t type);

void remap_depth(const k4a_image_t src, const k4a_image_t lut, k4a_image_t dst, interpolation_t type);
void remap_color(const k4a_image_t src, const k4a_image_t lut, k4a_image_t dst, interpolation_t type);

pinhole_t create_pinhole_from_xy_range(const k4a_calibration_t *calibration, const k4a_calibration_type_t camera);
pinhole_t create_pinhole_from_calib(const k4a_calibration_t *calibration, const k4a_calibration_type_t camera_type);

void write_calib(const std::string& filename, k4a_calibration_t calibration);

char* getCmdOption(char ** begin, char ** end, const std::string & option);

void split_filename (const std::string& str, std::string& folder, std::string& file);

bool color_to_depth(k4a_transformation_t transformation_handle,
                    const k4a_image_t depth_image,
                    const k4a_image_t color_image,
                    k4a_image_t& transformed_color_image,
                    const uint64_t dep_ts);

bool depth_to_color(k4a_transformation_t transformation_handle,
                    const k4a_image_t depth_image,
                    const k4a_image_t color_image,
                    k4a_image_t& transformed_depth_image,
                    const uint64_t dep_ts);

bool depth_ir_to_color(k4a_transformation_t transformation_handle,
                    const k4a_image_t depth_image,
                    const k4a_image_t ir_image,
                    const k4a_image_t color_image,
                    k4a_image_t& transformed_depth_image,
                    k4a_image_t& transformed_ir_image,
                    const uint64_t dep_ts);

k4a_image_t downscale_image_2x2_binning(const k4a_image_t color_image);

bool create_folder(const std::string& folderpath);

void getCamMatDistCoeffs(k4a_calibration_t& calib, const k4a_calibration_type_t& calib_type, cv::Mat& cam_mat, cv::Mat& dist_coeffs);

void transformation_helpers_write_point_cloud(const k4a_image_t point_cloud_image, const k4a_image_t color_image, const char *file_name);

#endif /* UTILS_H */
