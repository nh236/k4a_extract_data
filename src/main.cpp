#include <iostream>
#include <fstream>
#include <string>
#include <experimental/filesystem>

#include <k4a/k4a.h>
#include <k4arecord/types.h>
#include <k4arecord/playback.h>
#include "utils.h"
#include <cstdio>

#include <opencv2/opencv.hpp>

#include "utils.h"
#include "process_images.h"


////////////////////////
// extract_data
///////////////////////
int extract_data(const std::string& mkv_filepath, const int& rot_angle, const uint64_t& from_TS, const uint64_t& to_TS, const std::string& task_name, const bool& extract_ir, char * outfolder, const bool& use_global_TS)
{
    std::string foldername, filename;

    if (outfolder)
    {
        foldername = outfolder;
    }
    else
    {
        split_filename(mkv_filepath, foldername, filename);
        foldername += task_name;
    }

    if(!std::experimental::filesystem::exists(foldername))
    {
        create_folder(foldername);
    }
    
    k4a_playback_t playback_handle = NULL;
    k4a_image_t lut_color = NULL;
    k4a_calibration_t calib;

    k4a_capture_t capture = NULL;
    k4a_stream_result_t result = K4A_STREAM_RESULT_SUCCEEDED;
    interpolation_t interpolation_type = INTERPOLATION_NEARESTNEIGHBOR;

    int img_rot_flag = -1; // No rotation
    
    switch (rot_angle) {
        case 90:
            img_rot_flag = cv::ROTATE_90_CLOCKWISE;
            std::cout << "Rotating 90 degrees clockwise." << std::endl;
            break;
        case 180:
            img_rot_flag = cv::ROTATE_180;
            std::cout << "Rotate 180 degrees." << std::endl;
            break;
        case 270:
            img_rot_flag = cv::ROTATE_90_COUNTERCLOCKWISE;
            std::cout << "Rotating 90 degrees counterclockwise." << std::endl;
            break;
        default:
            std::cout << "No rotation. Angle = " << rot_angle << std::endl;
            img_rot_flag = -1;
            break;
        }


    pinhole_t pinhole_color;


    // Playback handle
    if (k4a_playback_open(mkv_filepath.c_str(), &playback_handle) != K4A_RESULT_SUCCEEDED)
    {
        printf("Failed to open recording\n");
        return 1;
    }

    // convert images to BGRA
    k4a_playback_set_color_conversion(playback_handle, K4A_IMAGE_FORMAT_COLOR_BGRA32);

    // get calibration
    if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(playback_handle, &calib))
    {
        printf("Failed to get calibration!\n");
    }

    if (WRITE_FILES)
    {
        // Create output folders
        // create_folder(foldername + "/rgb");
        // create_folder(foldername + "/depth");
        create_folder(foldername + "/downscaled");
        create_folder(foldername + "/downscaled/rgb");
        create_folder(foldername + "/downscaled/depth");
        if (extract_ir)
        {
            create_folder(foldername + "/downscaled/ir");
        }
        write_calib(foldername + "/calib.txt", calib);
    }

    k4a_transformation_t transformation_full = k4a_transformation_create(&calib);

    // ************* Downscale calibration/transformation ***********************3
    k4a_calibration_t calibration_color_downscaled;
    memcpy(&calibration_color_downscaled, &calib, sizeof(k4a_calibration_t));
    calibration_color_downscaled.color_camera_calibration.resolution_width /= 2;
    calibration_color_downscaled.color_camera_calibration.resolution_height /= 2;
    calibration_color_downscaled.color_camera_calibration.intrinsics.parameters.param.cx /= 2;
    calibration_color_downscaled.color_camera_calibration.intrinsics.parameters.param.cy /= 2;
    calibration_color_downscaled.color_camera_calibration.intrinsics.parameters.param.fx /= 2;
    calibration_color_downscaled.color_camera_calibration.intrinsics.parameters.param.fy /= 2;
    
    if (WRITE_FILES)
    {
        write_calib(foldername + "/downscaled/calib.txt", calibration_color_downscaled);
    }

    k4a_transformation_t transformation_color_downscaled = k4a_transformation_create(&calibration_color_downscaled);

    pinhole_t pinhole_color_downscaled = create_pinhole_from_calib(&calibration_color_downscaled, K4A_CALIBRATION_TYPE_COLOR);
    k4a_image_t lut_color_downscaled;

    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                     pinhole_color_downscaled.width,
                     pinhole_color_downscaled.height,
                     pinhole_color_downscaled.width * (int)sizeof(coordinate_t),
                     &lut_color_downscaled);

    create_undistortion_lut(&calibration_color_downscaled, K4A_CALIBRATION_TYPE_COLOR, &pinhole_color_downscaled, lut_color_downscaled, INTERPOLATION_NEARESTNEIGHBOR);

    pinhole_color = create_pinhole_from_calib(&calib, K4A_CALIBRATION_TYPE_COLOR);

    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                     pinhole_color.width,
                     pinhole_color.height,
                     pinhole_color.width * (int)sizeof(coordinate_t),
                     &lut_color);

    create_undistortion_lut(&calib, K4A_CALIBRATION_TYPE_COLOR, &pinhole_color, lut_color, INTERPOLATION_NEARESTNEIGHBOR);

    cv::Mat lookUpTable(1, 256, CV_8U);

    float gamma_ =.7;
        
    uchar* p = lookUpTable.ptr();
    for( int i = 0; i < 256; ++i)
        p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma_) * 255.0);
            
    // Statistics
    uint64_t recording_length = k4a_playback_get_recording_length_usec(playback_handle);
    printf("Recording is %lu seconds long\n", recording_length / 1000000);

    // Jump to from_TS frame!!
    k4a_record_configuration_t rec_conf;
    k4a_playback_get_record_configuration(playback_handle, &rec_conf);
    
    // Different options:
    // 1. from_TS and to_TS given relative to start of recording (use_global_TS == False)
    // 2. from_TS and to_TS given as absolute device timestamps (use_global_TS == True)
    k4a_playback_seek_origin_t seek_TS_type = K4A_PLAYBACK_SEEK_BEGIN;
    uint64_t start_timestamp_offset_usec = rec_conf.start_timestamp_offset_usec; // used for checking if current TS is between from_TS and to_TS

    if (use_global_TS)
    {
        seek_TS_type = K4A_PLAYBACK_SEEK_DEVICE_TIME;
        start_timestamp_offset_usec = 0;
    }

    uint64_t seek_TS_from = 0;
    // uint_64_t seek_TS_to; // not used

    if (rec_conf.start_timestamp_offset_usec > 0)
    {
        // store information on start timestamp offset (for other uses)
        std::ofstream tsfile;
        tsfile.open (foldername + "/start_timestamp_offset_usec.txt");
        tsfile << rec_conf.start_timestamp_offset_usec << "\n";
        tsfile.close();
    }

    seek_TS_from = from_TS;
    
    if (seek_TS_from > 0)
    {
        k4a_playback_seek_timestamp(playback_handle, seek_TS_from, seek_TS_type);
    }
    
    k4a_image_t last_rgb_im = NULL;
    bool done = false;
    
    // loop through data
    while (result == K4A_STREAM_RESULT_SUCCEEDED && !done)
    {
        result = k4a_playback_get_next_capture(playback_handle, &capture);
        if (result == K4A_STREAM_RESULT_SUCCEEDED)
        {
            // Get images
            k4a_image_t rgb_im = k4a_capture_get_color_image(capture);
            k4a_image_t dep_im = k4a_capture_get_depth_image(capture);
            k4a_image_t ir_im = NULL;
            
            if (extract_ir)
            {
                ir_im = k4a_capture_get_ir_image(capture);
            }


            if (rgb_im != NULL)
            {
                uint64_t rgb_ts = k4a_image_get_device_timestamp_usec(rgb_im);
                printf("RGB timestamp: %lu.\n", rgb_ts);
            }
            
            if (dep_im != NULL)
            {
                uint64_t dep_ts = k4a_image_get_device_timestamp_usec(dep_im);
                printf("Depth timestamp: %lu.\n", dep_ts);
            
                
                if (to_TS == std::numeric_limits<uint64_t>::max() || (dep_ts >= (from_TS + start_timestamp_offset_usec) && dep_ts <= (to_TS + start_timestamp_offset_usec) ))
                {
                    if (rgb_im != NULL)
                    {
                        process_images(dep_im, rgb_im, ir_im, calib, foldername, pinhole_color.width, pinhole_color.height,
                                    lut_color, lut_color_downscaled, interpolation_type, img_rot_flag,
                                    transformation_full, transformation_color_downscaled, lookUpTable);
                    }
                    else if (last_rgb_im != NULL)
                    {
                        printf("*** No RGB frame available - Using previous rgb frame!!! ***");
                        process_images(dep_im, last_rgb_im, ir_im, calib, foldername, pinhole_color.width, pinhole_color.height,
                                    lut_color, lut_color_downscaled, interpolation_type, img_rot_flag,
                                    transformation_full, transformation_color_downscaled, lookUpTable);
                    }
                    else
                    {
                        printf("******* Async depth/rgb!!! *****************");
                    }
                }
                else if (dep_ts > to_TS + start_timestamp_offset_usec)
                {
                    done = true;
                }
                
            }

            // release everything
            if (dep_im != NULL)
            {
                k4a_image_release(dep_im);
            }

            if (rgb_im != NULL)
            {
                if (last_rgb_im != NULL)
                {
                    k4a_image_release(last_rgb_im);
                }
                
                last_rgb_im = k4a_capture_get_color_image(capture);
                
                k4a_image_release(rgb_im);
            }

            k4a_capture_release(capture);
        }
        else if (result == K4A_STREAM_RESULT_EOF)
        {
            // End of file reached
            break;
        }
    }
    
    if (last_rgb_im != NULL)
    {
        k4a_image_release(last_rgb_im);
    }

    if (result == K4A_STREAM_RESULT_FAILED)
    {
        printf("Failed to read entire recording\n");
        return 1;
    }

    k4a_transformation_destroy(transformation_full);
    k4a_transformation_destroy(transformation_color_downscaled);

    k4a_image_release(lut_color_downscaled);
    k4a_image_release(lut_color);
    k4a_playback_close(playback_handle);

    return 0;
}


////////////////////////
// main
///////////////////////
int main(int argc, char** argv)
{
    if (argc < 2)
    {
        printf("Usage: k4a_extract_data <mkv_filename> [<rotation angle (90, 180, 270), default: 0>] [<from_TS> <to_TS> <task_name>] [--outfolder <output_folder>] [--global_TS] [--ir]\n");
        return 2;
    }


    std::string mkv_filepath = argv[1];


    int rot_angle = 0;
    uint64_t from_TS = 0;
    uint64_t to_TS = std::numeric_limits<uint64_t>::max();
    std::string task_name = "";
    bool extract_ir = getCmdOption(argv, argv + argc, "--ir");;
    char * outfolder = getCmdOption(argv, argv + argc, "--outfolder");
    bool use_global_TS = cmdOptionExists(argv, argv+argc, "--global_TS"); // whether to consider timestamps (from_TS, to_TS) as global timestamps or relative to beginning of recording

    if (argc > 2)
    {
        rot_angle = atoi(argv[2]);
        std::cout << "Rotation angle: " << rot_angle << std::endl;
    }

    if (argc >= 6)
    {
        from_TS = atol(argv[3]);
        to_TS = atol(argv[4]);
        task_name = "_" + std::string(argv[5]);
        std::cout << "from_TS: " << from_TS << ", to_TS: " << to_TS << ", task_name: " << task_name << ", global_TS: " << use_global_TS << std::endl;
    }
    

    extract_data(mkv_filepath, rot_angle, from_TS, to_TS, task_name, extract_ir, outfolder, use_global_TS);

    return 0;
}
