
#ifndef PROCESS_IMAGES
#define PROCESS_IMAGES

#include <k4a/k4a.h>
#include <opencv2/opencv.hpp>
#include "utils.h"

void process_images(const k4a_image_t& dep_im, 
                    const k4a_image_t& rgb_im,
                    const k4a_image_t& ir_im,
                    k4a_calibration_t& calibration,
                    const std::string& foldername,
                    const int out_wid, const int out_hgt,
                    const k4a_image_t& lut_color, 
                    const k4a_image_t& lut_color_downscaled,
                    interpolation_t& interpolation_type, 
                    const int& img_rot_flag,
                    const k4a_transformation_t& transformation_full,
                    const k4a_transformation_t& transformation_color_downscaled,
                    const cv::Mat& lut_gamma
                   );

#endif /* PROCESS_IMAGES */
