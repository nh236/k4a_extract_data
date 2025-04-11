#include "process_images.h"

#include <string>
#include <experimental/filesystem>

void process_images(const k4a_image_t& dep_im, const k4a_image_t& rgb_im, const k4a_image_t& ir_im, k4a_calibration_t& calibration,
                    const std::string& foldername, const int out_wid, const int out_hgt,
                    const k4a_image_t& lut_color, const k4a_image_t& lut_color_downscaled,
                    interpolation_t& interpolation_type, const int& img_rot_flag,
                    const k4a_transformation_t& transformation_full, const k4a_transformation_t& transformation_color_downscaled,
                    const cv::Mat& lut_gamma)
{
    uint64_t dep_ts = k4a_image_get_device_timestamp_usec(dep_im);

    char depth_ts_str[256];
    sprintf(depth_ts_str, "%012lu", dep_ts);

    char rgb_filename[512], rgb_downscaled_filename[512], depth_filename[512], depth_downscaled_filename[512],ir_filename[512], ir_downscaled_filename[512];


    if (WRITE_FILES)
    {
        sprintf(rgb_filename, "%s/rgb/%s.png", foldername.c_str(), depth_ts_str);
        sprintf(depth_filename, "%s/depth/%s_depth.png", foldername.c_str(), depth_ts_str);
        sprintf(ir_filename, "%s/ir/%s_ir.png", foldername.c_str(), depth_ts_str);
        sprintf(rgb_downscaled_filename, "%s/downscaled/rgb/%s.png", foldername.c_str(), depth_ts_str);
        sprintf(depth_downscaled_filename, "%s/downscaled/depth/%s_depth.png", foldername.c_str(), depth_ts_str);
        sprintf(ir_downscaled_filename, "%s/downscaled/ir/%s_ir.png", foldername.c_str(), depth_ts_str);
    }
    
    // Skip if files exist
    if (!std::experimental::filesystem::exists(rgb_downscaled_filename) ||
            !std::experimental::filesystem::exists(depth_downscaled_filename) ||
       (ir_im != NULL && !std::experimental::filesystem::exists(ir_downscaled_filename))
        
    )
    {
        k4a_image_t undistorted_depth = NULL;
        k4a_image_t undistorted_ir = NULL;
        k4a_image_t undistorted_color = NULL;
        k4a_image_t transformed_ir_image = NULL;
        k4a_image_t undistorted_ir_downscaled = NULL;
        k4a_image_t transformed_depth_image, undistorted_color_downscaled, undistorted_depth_downscaled;

//         int depth_image_width_pixels = k4a_image_get_width_pixels(dep_im);
//         int depth_image_height_pixels = k4a_image_get_height_pixels(dep_im);
//         int color_image_width_pixels = k4a_image_get_width_pixels(rgb_im);
//         int color_image_height_pixels = k4a_image_get_height_pixels(rgb_im);

        // ************************ Undistort, rotate and write color image ************************
        k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                         out_wid,
                         out_hgt,
                         out_wid * 4 * (int)sizeof(uint8_t),
                         &undistorted_color);

        remap_color(rgb_im, lut_color, undistorted_color, interpolation_type);

//         cv::Mat undistColorMat(out_hgt, out_wid, CV_8UC4, (void *) k4a_image_get_buffer(undistorted_color), cv::Mat::AUTO_STEP);
// 
//         if (img_rot_flag == cv::ROTATE_90_CLOCKWISE ||
//                 img_rot_flag == cv::ROTATE_90_COUNTERCLOCKWISE ||
//                 img_rot_flag == cv::ROTATE_180)
//         {
//             cv::rotate(undistColorMat, undistColorMat, img_rot_flag);
//         }
// 
//         full scale undistorted color image
//         cv::imwrite(rgb_filename, undistColorMat);
        

        // ************************ Transform depth + ir to color ************************
        if (ir_im != NULL)
        {
            depth_ir_to_color(transformation_full, dep_im, ir_im, rgb_im, transformed_depth_image, transformed_ir_image, dep_ts);
            k4a_image_create(K4A_IMAGE_FORMAT_IR16,
                         out_wid,
                         out_hgt,
                         out_wid * (int)sizeof(uint16_t),
                         &undistorted_ir);
            remap_depth(transformed_ir_image, lut_color, undistorted_ir, interpolation_type);

            cv::Mat undistIRMat(k4a_image_get_height_pixels(undistorted_ir), k4a_image_get_width_pixels(undistorted_ir), CV_16UC1, (void *) k4a_image_get_buffer(undistorted_ir), cv::Mat::AUTO_STEP);
            
            if (img_rot_flag == cv::ROTATE_90_CLOCKWISE || img_rot_flag == cv::ROTATE_90_COUNTERCLOCKWISE || img_rot_flag == cv::ROTATE_180)
            {
                cv::rotate(undistIRMat, undistIRMat, img_rot_flag);
            }
        }

        depth_to_color(transformation_full, dep_im, rgb_im, transformed_depth_image, dep_ts);
        
        
        k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                         out_wid,
                         out_hgt,
                         out_wid * (int)sizeof(uint16_t),
                         &undistorted_depth);

        // ************************ Undistort (color cam lut now!!) ************************
        remap_depth(transformed_depth_image, lut_color, undistorted_depth, interpolation_type);


        cv::Mat undistDepMat(k4a_image_get_height_pixels(undistorted_depth), k4a_image_get_width_pixels(undistorted_depth), CV_16UC1, (void *) k4a_image_get_buffer(undistorted_depth), cv::Mat::AUTO_STEP);
        
        if (img_rot_flag == cv::ROTATE_90_CLOCKWISE || img_rot_flag == cv::ROTATE_90_COUNTERCLOCKWISE || img_rot_flag == cv::ROTATE_180)
        {
            cv::rotate(undistDepMat, undistDepMat, img_rot_flag);
        }
        
        // full scale undistorted depth image
        //cv::imwrite(depth_filename, undistDepMat);

        // ************************ Downscale ************************
        k4a_image_t color_image_downscaled = downscale_image_2x2_binning(rgb_im);

        int rgb_width_downscaled = k4a_image_get_width_pixels(color_image_downscaled);
        int rgb_height_downscaled = k4a_image_get_height_pixels(color_image_downscaled);


        // ************************ Undistort and write downscaled color image ************************
        k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                         rgb_width_downscaled,
                         rgb_height_downscaled,
                         rgb_width_downscaled * 4 * (int)sizeof(uint8_t),
                         &undistorted_color_downscaled);

        remap_color(color_image_downscaled, lut_color_downscaled, undistorted_color_downscaled, interpolation_type);

        cv::Mat undistColorMat_downscaled(rgb_height_downscaled, rgb_width_downscaled, CV_8UC4,
                                          (void *) k4a_image_get_buffer(undistorted_color_downscaled), cv::Mat::AUTO_STEP);
        
        if (img_rot_flag == cv::ROTATE_90_CLOCKWISE || img_rot_flag == cv::ROTATE_90_COUNTERCLOCKWISE || img_rot_flag == cv::ROTATE_180)
        {
            cv::rotate(undistColorMat_downscaled, undistColorMat_downscaled, img_rot_flag);
        }
        
        if (WRITE_FILES && !std::experimental::filesystem::exists(rgb_downscaled_filename))
        {
            if (DO_GAMMA_CORRECT)
            {
                LUT(undistColorMat_downscaled, lut_gamma, undistColorMat_downscaled);
            }
            
            cv::imwrite(rgb_downscaled_filename, undistColorMat_downscaled);
        }


        // ************************ Transform depth to color, undistort, write ************************
        k4a_image_t transformed_depth_downscaled = NULL;
        if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                rgb_width_downscaled,
                rgb_height_downscaled,
                rgb_width_downscaled * (int)sizeof(uint16_t),
                &transformed_depth_downscaled))
        {
            printf("Failed to create transformed depth image\n");
        }

        if (K4A_RESULT_SUCCEEDED !=
                k4a_transformation_depth_image_to_color_camera(transformation_color_downscaled, dep_im, transformed_depth_downscaled))
        {
            printf("Failed to compute transformed depth image\n");
        }

        k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                         rgb_width_downscaled,
                         rgb_height_downscaled,
                         rgb_width_downscaled * (int)sizeof(uint16_t),
                         &undistorted_depth_downscaled);

        // ************************ Undistort (color cam lut now!!) ************************
        remap_depth(transformed_depth_downscaled, lut_color_downscaled, undistorted_depth_downscaled, interpolation_type);


        cv::Mat undistDepMat_downscaled(k4a_image_get_height_pixels(undistorted_depth_downscaled), k4a_image_get_width_pixels(undistorted_depth_downscaled), CV_16UC1, (void *) k4a_image_get_buffer(undistorted_depth_downscaled), cv::Mat::AUTO_STEP);

        if (img_rot_flag == cv::ROTATE_90_CLOCKWISE || img_rot_flag == cv::ROTATE_90_COUNTERCLOCKWISE || img_rot_flag == cv::ROTATE_180)
        {        
            cv::rotate(undistDepMat_downscaled, undistDepMat_downscaled, img_rot_flag);
        }
        
        if (WRITE_FILES && !std::experimental::filesystem::exists(depth_downscaled_filename))
        {
            cv::imwrite(depth_downscaled_filename, undistDepMat_downscaled);
        }

        k4a_image_t transformed_ir_downscaled = NULL;
        if (ir_im != NULL)
        {
            if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_IR16,
                    rgb_width_downscaled,
                    rgb_height_downscaled,
                    rgb_width_downscaled * (int)sizeof(uint16_t),
                    &transformed_ir_downscaled))
            {
                printf("Failed to create transformed ir image\n");
            }

            if (K4A_RESULT_SUCCEEDED != depth_ir_to_color(transformation_color_downscaled, dep_im, ir_im, color_image_downscaled, transformed_depth_downscaled, transformed_ir_downscaled, dep_ts))
                    // k4a_transformation_depth_image_to_color_camera(transformation_color_downscaled, dep_im, transformed_depth_downscaled))
            {
                printf("Failed to compute transformed ir image\n");
            }

            k4a_image_create(K4A_IMAGE_FORMAT_IR16,
                            rgb_width_downscaled,
                            rgb_height_downscaled,
                            rgb_width_downscaled * (int)sizeof(uint16_t),
                            &undistorted_ir_downscaled);

            // ************************ Undistort (color cam lut now!!) ************************
            remap_depth(transformed_ir_downscaled, lut_color_downscaled, undistorted_ir_downscaled, interpolation_type);


            cv::Mat undistIRMat_downscaled(k4a_image_get_height_pixels(undistorted_ir_downscaled), k4a_image_get_width_pixels(undistorted_ir_downscaled), CV_16UC1, (void *) k4a_image_get_buffer(undistorted_ir_downscaled), cv::Mat::AUTO_STEP);

            if (img_rot_flag == cv::ROTATE_90_CLOCKWISE || img_rot_flag == cv::ROTATE_90_COUNTERCLOCKWISE || img_rot_flag == cv::ROTATE_180)
            {        
                cv::rotate(undistIRMat_downscaled, undistIRMat_downscaled, img_rot_flag);
            }
            
            if (WRITE_FILES && !std::experimental::filesystem::exists(ir_downscaled_filename))
            {
                cv::imwrite(ir_downscaled_filename, undistIRMat_downscaled);
            }
        }

        k4a_image_release(undistorted_color_downscaled);
        k4a_image_release(undistorted_depth_downscaled);
        k4a_image_release(color_image_downscaled);
        k4a_image_release(transformed_depth_downscaled);
        k4a_image_release(transformed_depth_image);
        k4a_image_release(undistorted_depth);
        k4a_image_release(undistorted_color);
                
        if (undistorted_ir_downscaled != NULL)
        {
            k4a_image_release(undistorted_ir_downscaled);
        }
        
        if (transformed_ir_downscaled != NULL)
        {
            k4a_image_release(transformed_ir_downscaled);
        }
        
        if (transformed_ir_image != NULL)
        {
            k4a_image_release(transformed_ir_image);
        }
        
        if (undistorted_ir != NULL)
        {
            k4a_image_release(undistorted_ir);
        }
    }
}
