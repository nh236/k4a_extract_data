// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <k4a/k4a.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <algorithm>

#include <experimental/filesystem>

#include "utils.h"

#define INVALID INT32_MIN


char* getCmdOption(char ** begin, char ** end, const std::string & option)
{
    char ** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}


bool cmdOptionExists(char** begin, char** end, const std::string& option)
{
    return std::find(begin, end, option) != end;
}


void split_filename (const std::string& str, std::string& folder, std::string& file)
{
    size_t found;

    found=str.find_last_of("/\\");

    if (found)
    {
        folder = str.substr(0,found);
        file = str.substr(found+1);

        std::cout << "Folder: " << folder << ", file: " << file << std::endl;
    }
}


////////////////////////
// write_calib
///////////////////////
void write_calib(const std::string& filename, k4a_calibration_t calibration)
{
    cv::Mat se3 =
        cv::Mat(3, 3, CV_32FC1, calibration.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation);
    cv::Mat r_vec = cv::Mat(3, 1, CV_32FC1);
    Rodrigues(se3, r_vec);
    cv::Mat t_vec =
        cv::Mat(3, 1, CV_32F, calibration.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].translation);

        
    cv::Mat se3_d2c =
        cv::Mat(3, 3, CV_32FC1, calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation);
    cv::Mat r_vec_d2c = cv::Mat(3, 1, CV_32FC1);
    Rodrigues(se3_d2c, r_vec_d2c);
    cv::Mat t_vec_d2c =
        cv::Mat(3, 1, CV_32F, calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].translation);
        
    // intrinsic parameters of the depth camera
    k4a_calibration_intrinsic_parameters_t *intrinsics = &calibration.depth_camera_calibration.intrinsics.parameters;
    std::vector<float> _camera_matrix = {intrinsics->param.fx, 0.f, intrinsics->param.cx, 0.f, intrinsics->param.fy, intrinsics->param.cy, 0.f, 0.f, 1.f};
    cv::Mat camera_matrix = cv::Mat(3, 3, CV_32F, &_camera_matrix[0]);
    std::vector<float> _dist_coeffs = { intrinsics->param.k1, intrinsics->param.k2, intrinsics->param.p1,
                                        intrinsics->param.p2, intrinsics->param.k3, intrinsics->param.k4,
                                        intrinsics->param.k5, intrinsics->param.k6
                                      };
    cv::Mat dist_coeffs = cv::Mat(8, 1, CV_32F, &_dist_coeffs[0]);


    // intrinsic parameters of the color camera
    k4a_calibration_intrinsic_parameters_t *color_intrinsics = &calibration.color_camera_calibration.intrinsics.parameters;
    std::vector<float> _color_camera_matrix = {color_intrinsics->param.fx, 0.f, color_intrinsics->param.cx,
                                               0.f, color_intrinsics->param.fy, color_intrinsics->param.cy,
                                               0.f, 0.f, 1.f
                                              };

    cv::Mat color_camera_matrix = cv::Mat(3, 3, CV_32F, &_color_camera_matrix[0]);
    std::vector<float> _color_dist_coeffs = { color_intrinsics->param.k1, color_intrinsics->param.k2, color_intrinsics->param.p1,
                                              color_intrinsics->param.p2, color_intrinsics->param.k3, color_intrinsics->param.k4,
                                              color_intrinsics->param.k5, color_intrinsics->param.k6
                                            };
    cv::Mat color_dist_coeffs = cv::Mat(8, 1, CV_32F, &_color_dist_coeffs[0]);

    cv::FileStorage file(filename, cv::FileStorage::WRITE);
    file << "color_depth_rot" << se3;
    file << "color_depth_trans" << t_vec;
    file << "depth_color_rot" << se3_d2c;
    file << "depth_color_trans" << t_vec_d2c;
    file << "depth_cam_matrix" << camera_matrix;
    file << "depth_dist_coeffs" << dist_coeffs;
    file << "color_cam_matrix" << color_camera_matrix;
    file << "color_dist_coeffs" << color_dist_coeffs;
}


////////////////////////
// depth_to_color
///////////////////////
bool depth_to_color(k4a_transformation_t transformation_handle,
                    const k4a_image_t depth_image,
                    const k4a_image_t color_image,
                    k4a_image_t& transformed_depth_image,
                    const uint64_t dep_ts)
{
    // transform color image into depth camera geometry
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
        
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
            color_image_width_pixels,
            color_image_height_pixels,
            color_image_width_pixels * (int)sizeof(uint16_t),
            &transformed_depth_image))
    {
        printf("Failed to create transformed depth image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED !=
            k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image))
    {
        printf("Failed to compute transformed depth image\n");
        return false;
    }

    //cv::Mat depthMat(color_image_height_pixels, color_image_width_pixels, CV_16UC1, (void *) k4a_image_get_buffer(transformed_depth_image), cv::Mat::AUTO_STEP);
    //cv::imwrite("")

    //cv::imwrite(foldername + "/depth/" + std::to_string(dep_ts) + "_depth.png", depthMat);

    //cv::imshow("Bla", depthMat);
    //cv::waitKey(0);
    //cv::destroyAllWindows();

    //k4a_image_release(transformed_depth_image);

    return true;
}


////////////////////////
// ir_to_color
///////////////////////
bool depth_ir_to_color(k4a_transformation_t transformation_handle,
                    const k4a_image_t depth_image,
                    const k4a_image_t ir_image,
                    const k4a_image_t color_image,
                    k4a_image_t& transformed_depth_image,
                    k4a_image_t& transformed_ir_image,
                    const uint64_t dep_ts)
{
    // transform color image into depth camera geometry
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
        
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM16,
            color_image_width_pixels,
            color_image_height_pixels,
            color_image_width_pixels * (int)sizeof(uint16_t),
            &transformed_ir_image))
    {
        printf("Failed to create transformed ir image\n");
        return false;
    }

        if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
            color_image_width_pixels,
            color_image_height_pixels,
            color_image_width_pixels * (int)sizeof(uint16_t),
            &transformed_depth_image))
    {
        printf("Failed to create transformed depth image\n");
        return false;
    }
    
    k4a_image_t custom_ir_image = NULL;
    int ir_image_width_pixels = k4a_image_get_width_pixels(ir_image);
    int ir_image_height_pixels = k4a_image_get_height_pixels(ir_image);
    int ir_image_stride_bytes = k4a_image_get_stride_bytes(ir_image);
    uint8_t *ir_image_buffer = k4a_image_get_buffer(ir_image);
    
    k4a_image_create_from_buffer(K4A_IMAGE_FORMAT_CUSTOM16,
                             ir_image_width_pixels,
                             ir_image_height_pixels,
                             ir_image_stride_bytes,
                             ir_image_buffer,
                             ir_image_height_pixels * ir_image_stride_bytes,
                             [](void *_buffer, void *context) {
                                  delete[](uint8_t *) _buffer;
                                  (void)context;
                             },
                             NULL,
                             &custom_ir_image);
    
    
    if (K4A_RESULT_SUCCEEDED !=
            k4a_transformation_depth_image_to_color_camera_custom(transformation_handle, depth_image,
                                                                  custom_ir_image,
                                                                  transformed_depth_image,
                                                                  transformed_ir_image,
                                                                  K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST,
                                                                  0
                                                                 ))
    {
        printf("Failed to compute transformed depth image\n");
        return false;
    }

    //cv::Mat depthMat(color_image_height_pixels, color_image_width_pixels, CV_16UC1, (void *) k4a_image_get_buffer(transformed_depth_image), cv::Mat::AUTO_STEP);
    //cv::imwrite("")

    //cv::imwrite(foldername + "/depth/" + std::to_string(dep_ts) + "_depth.png", depthMat);

    //cv::imshow("Bla", depthMat);
    //cv::waitKey(0);
    //cv::destroyAllWindows();

    //k4a_image_release(transformed_depth_image);

    return K4A_RESULT_SUCCEEDED;
}

////////////////////////
// color_to_depth
///////////////////////
bool color_to_depth(k4a_transformation_t transformation_handle,
                    const k4a_image_t depth_image,
                    const k4a_image_t color_image,
                    k4a_image_t& transformed_color_image,
                    const uint64_t dep_ts)
{
    // transform color image into depth camera geometry
    int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
    int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);
    transformed_color_image = NULL;

    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
            depth_image_width_pixels,
            depth_image_height_pixels,
            depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
            &transformed_color_image))
    {
        printf("Failed to create transformed color image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED !=
            k4a_transformation_color_image_to_depth_camera(transformation_handle, depth_image, color_image, transformed_color_image))
    {
        printf("Failed to compute transformed color image\n");
        return false;
    }

    return true;
}

// Compute a conservative bounding box on the unit plane in which all the points have valid projections
void compute_xy_range(const k4a_calibration_t *calibration,
                             const k4a_calibration_type_t camera,
                             const int width,
                             const int height,
                             float &x_min,
                             float &x_max,
                             float &y_min,
                             float &y_max)
{
    // Step outward from the centre point until we find the bounds of valid projection
    const float step_u = 0.25f;
    const float step_v = 0.25f;
    const float min_u = 0;
    const float min_v = 0;
    const float max_u = (float)width - 1;
    const float max_v = (float)height - 1;
    const float center_u = 0.5f * width;
    const float center_v = 0.5f * height;

    int valid;
    k4a_float2_t p;
    k4a_float3_t ray;

    // search x_min
    for (float uv[2] = { center_u, center_v }; uv[0] >= min_u; uv[0] -= step_u)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        x_min = ray.xyz.x;
    }

    // search x_max
    for (float uv[2] = { center_u, center_v }; uv[0] <= max_u; uv[0] += step_u)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        x_max = ray.xyz.x;
    }

    // search y_min
    for (float uv[2] = { center_u, center_v }; uv[1] >= min_v; uv[1] -= step_v)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        y_min = ray.xyz.y;
    }

    // search y_max
    for (float uv[2] = { center_u, center_v }; uv[1] <= max_v; uv[1] += step_v)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        y_max = ray.xyz.y;
    }
}

pinhole_t create_pinhole_from_xy_range(const k4a_calibration_t *calibration, const k4a_calibration_type_t camera)
{
    int width = calibration->depth_camera_calibration.resolution_width;
    int height = calibration->depth_camera_calibration.resolution_height;
    if (camera == K4A_CALIBRATION_TYPE_COLOR)
    {
        width = calibration->color_camera_calibration.resolution_width;
        height = calibration->color_camera_calibration.resolution_height;
    }

    float x_min = 0, x_max = 0, y_min = 0, y_max = 0;
    compute_xy_range(calibration, camera, width, height, x_min, x_max, y_min, y_max);

    pinhole_t pinhole;

    float fx = 1.f / (x_max - x_min);
    float fy = 1.f / (y_max - y_min);
    float px = -x_min * fx;
    float py = -y_min * fy;

    pinhole.fx = fx * width;
    pinhole.fy = fy * height;
    pinhole.px = px * width;
    pinhole.py = py * height;
    pinhole.width = width;
    pinhole.height = height;

    return pinhole;
}


pinhole_t create_pinhole_from_calib(const k4a_calibration_t *calibration, const k4a_calibration_type_t camera_type)
{
    k4a_calibration_camera_t calib_cam;
    
    if (camera_type == K4A_CALIBRATION_TYPE_COLOR)
    {
        calib_cam = calibration->color_camera_calibration;
    }
    else if (camera_type == K4A_CALIBRATION_TYPE_DEPTH)
    {
        calib_cam = calibration->depth_camera_calibration;
    }
    else
    {
        printf("ERROR: WRONG CALIBRATION TYPE FOR PINHOLE CAMERA CREATION!");
    }

    
    pinhole_t pinhole;

    pinhole.fx = calib_cam.intrinsics.parameters.param.fx;;
    pinhole.fy = calib_cam.intrinsics.parameters.param.fy;
    pinhole.px = calib_cam.intrinsics.parameters.param.cx;
    pinhole.py = calib_cam.intrinsics.parameters.param.cy;
    pinhole.width = calib_cam.resolution_width;
    pinhole.height = calib_cam.resolution_height;

    return pinhole;
}


void create_undistortion_lut(const k4a_calibration_t *calibration,
                                    const k4a_calibration_type_t camera,
                                    const pinhole_t *pinhole,
                                    k4a_image_t lut,
                                    interpolation_t type)
{
    coordinate_t *lut_data = (coordinate_t *)(void *)k4a_image_get_buffer(lut);

    k4a_float3_t ray;
    ray.xyz.z = 1.f;

    int src_width = calibration->depth_camera_calibration.resolution_width;
    int src_height = calibration->depth_camera_calibration.resolution_height;
    if (camera == K4A_CALIBRATION_TYPE_COLOR)
    {
        src_width = calibration->color_camera_calibration.resolution_width;
        src_height = calibration->color_camera_calibration.resolution_height;
    }

    for (int y = 0, idx = 0; y < pinhole->height; y++)
    {
        ray.xyz.y = ((float)y - pinhole->py) / pinhole->fy;

        for (int x = 0; x < pinhole->width; x++, idx++)
        {
            ray.xyz.x = ((float)x - pinhole->px) / pinhole->fx;

            k4a_float2_t distorted;
            int valid;
            k4a_calibration_3d_to_2d(calibration, &ray, camera, camera, &distorted, &valid);

            coordinate_t src;
            if (type == INTERPOLATION_NEARESTNEIGHBOR)
            {
                // Remapping via nearest neighbor interpolation
                src.x = (int)floorf(distorted.xy.x + 0.5f);
                src.y = (int)floorf(distorted.xy.y + 0.5f);
            }
            else if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
            {
                // Remapping via bilinear interpolation
                src.x = (int)floorf(distorted.xy.x);
                src.y = (int)floorf(distorted.xy.y);
            }
            else
            {
                printf("Unexpected interpolation type!\n");
                exit(-1);
            }

            if (valid && src.x >= 0 && src.x < src_width && src.y >= 0 && src.y < src_height)
            {
                lut_data[idx] = src;

                if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
                {
                    // Compute the floating point weights, using the distance from projected point src to the
                    // image coordinate of the upper left neighbor
                    float w_x = distorted.xy.x - src.x;
                    float w_y = distorted.xy.y - src.y;
                    float w0 = (1.f - w_x) * (1.f - w_y);
                    float w1 = w_x * (1.f - w_y);
                    float w2 = (1.f - w_x) * w_y;
                    float w3 = w_x * w_y;

                    // Fill into lut
                    lut_data[idx].weight[0] = w0;
                    lut_data[idx].weight[1] = w1;
                    lut_data[idx].weight[2] = w2;
                    lut_data[idx].weight[3] = w3;
                }
            }
            else
            {
                lut_data[idx].x = INVALID;
                lut_data[idx].y = INVALID;
            }
        }
    }
}


void remap_depth(const k4a_image_t src, const k4a_image_t lut, k4a_image_t dst, interpolation_t type)
{
    int src_width = k4a_image_get_width_pixels(src);
    int dst_width = k4a_image_get_width_pixels(dst);
    int dst_height = k4a_image_get_height_pixels(dst);

    uint16_t *src_data = (uint16_t *)(void *)k4a_image_get_buffer(src);
    uint16_t *dst_data = (uint16_t *)(void *)k4a_image_get_buffer(dst);
    coordinate_t *lut_data = (coordinate_t *)(void *)k4a_image_get_buffer(lut);

    memset(dst_data, 0, (size_t)dst_width * (size_t)dst_height * sizeof(uint16_t));

    for (int i = 0; i < dst_width * dst_height; i++)
    {
        if (lut_data[i].x != INVALID && lut_data[i].y != INVALID)
        {
            if (type == INTERPOLATION_NEARESTNEIGHBOR)
            {
                dst_data[i] = src_data[lut_data[i].y * src_width + lut_data[i].x];
            }
            else if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
            {
                const uint16_t neighbors[4]{ src_data[lut_data[i].y * src_width + lut_data[i].x],
                                             src_data[lut_data[i].y * src_width + lut_data[i].x + 1],
                                             src_data[(lut_data[i].y + 1) * src_width + lut_data[i].x],
                                             src_data[(lut_data[i].y + 1) * src_width + lut_data[i].x + 1] };

                if (type == INTERPOLATION_BILINEAR_DEPTH)
                {
                    // If the image contains invalid data, e.g. depth image contains value 0, ignore the bilinear
                    // interpolation for current target pixel if one of the neighbors contains invalid data to avoid
                    // introduce noise on the edge. If the image is color or ir images, user should use
                    // INTERPOLATION_BILINEAR
                    if (neighbors[0] == 0 || neighbors[1] == 0 || neighbors[2] == 0 || neighbors[3] == 0)
                    {
                        continue;
                    }

                    // Ignore interpolation at large depth discontinuity without disrupting slanted surface
                    // Skip interpolation threshold is estimated based on the following logic:
                    // - angle between two pixels is: theta = 0.234375 degree (120 degree / 512) in binning resolution
                    // mode
                    // - distance between two pixels at same depth approximately is: A ~= sin(theta) * depth
                    // - distance between two pixels at highly slanted surface (e.g. alpha = 85 degree) is: B = A /
                    // cos(alpha)
                    // - skip_interpolation_ratio ~= sin(theta) / cos(alpha)
                    // We use B as the threshold that to skip interpolation if the depth difference in the triangle is
                    // larger than B. This is a conservative threshold to estimate largest distance on a highly slanted
                    // surface at given depth, in reality, given distortion, distance, resolution difference, B can be
                    // smaller
                    const float skip_interpolation_ratio = 0.04693441759f;
                    float depth_min = std::min(std::min(neighbors[0], neighbors[1]),
                                               std::min(neighbors[2], neighbors[3]));
                    float depth_max = std::max(std::max(neighbors[0], neighbors[1]),
                                               std::max(neighbors[2], neighbors[3]));
                    float depth_delta = depth_max - depth_min;
                    float skip_interpolation_threshold = skip_interpolation_ratio * depth_min;
                    if (depth_delta > skip_interpolation_threshold)
                    {
                        continue;
                    }
                }

                dst_data[i] = (uint16_t)(neighbors[0] * lut_data[i].weight[0] + neighbors[1] * lut_data[i].weight[1] +
                                         neighbors[2] * lut_data[i].weight[2] + neighbors[3] * lut_data[i].weight[3] +
                                         0.5f);
            }
            else
            {
                printf("Unexpected interpolation type!\n");
                exit(-1);
            }
        }
    }
}

void remap_color(const k4a_image_t src, const k4a_image_t lut, k4a_image_t dst, interpolation_t type)
{
    int src_width = k4a_image_get_width_pixels(src);
    int dst_width = k4a_image_get_width_pixels(dst);
    int dst_height = k4a_image_get_height_pixels(dst);

    uint32_t *src_data = (uint32_t *)(void *)k4a_image_get_buffer(src);
    uint32_t *dst_data = (uint32_t *)(void *)k4a_image_get_buffer(dst);
    coordinate_t *lut_data = (coordinate_t *)(void *)k4a_image_get_buffer(lut);

    memset(dst_data, 0, (size_t)dst_width * (size_t)dst_height * sizeof(uint32_t));

    for (int i = 0; i < dst_width * dst_height; i++)
    {
        if (lut_data[i].x != INVALID && lut_data[i].y != INVALID)
        {
            if (type == INTERPOLATION_NEARESTNEIGHBOR)
            {
                dst_data[i] = src_data[lut_data[i].y * src_width + lut_data[i].x];
            }
            else if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
            {
                const uint32_t neighbors[4]{ src_data[lut_data[i].y * src_width + lut_data[i].x],
                                             src_data[lut_data[i].y * src_width + lut_data[i].x + 1],
                                             src_data[(lut_data[i].y + 1) * src_width + lut_data[i].x],
                                             src_data[(lut_data[i].y + 1) * src_width + lut_data[i].x + 1] };

                if (type == INTERPOLATION_BILINEAR_DEPTH)
                {
                    // If the image contains invalid data, e.g. depth image contains value 0, ignore the bilinear
                    // interpolation for current target pixel if one of the neighbors contains invalid data to avoid
                    // introduce noise on the edge. If the image is color or ir images, user should use
                    // INTERPOLATION_BILINEAR
                    if (neighbors[0] == 0 || neighbors[1] == 0 || neighbors[2] == 0 || neighbors[3] == 0)
                    {
                        continue;
                    }

                    // Ignore interpolation at large depth discontinuity without disrupting slanted surface
                    // Skip interpolation threshold is estimated based on the following logic:
                    // - angle between two pixels is: theta = 0.234375 degree (120 degree / 512) in binning resolution
                    // mode
                    // - distance between two pixels at same depth approximately is: A ~= sin(theta) * depth
                    // - distance between two pixels at highly slanted surface (e.g. alpha = 85 degree) is: B = A /
                    // cos(alpha)
                    // - skip_interpolation_ratio ~= sin(theta) / cos(alpha)
                    // We use B as the threshold that to skip interpolation if the depth difference in the triangle is
                    // larger than B. This is a conservative threshold to estimate largest distance on a highly slanted
                    // surface at given depth, in reality, given distortion, distance, resolution difference, B can be
                    // smaller
                    const float skip_interpolation_ratio = 0.04693441759f;
                    float depth_min = std::min(std::min(neighbors[0], neighbors[1]),
                                               std::min(neighbors[2], neighbors[3]));
                    float depth_max = std::max(std::max(neighbors[0], neighbors[1]),
                                               std::max(neighbors[2], neighbors[3]));
                    float depth_delta = depth_max - depth_min;
                    float skip_interpolation_threshold = skip_interpolation_ratio * depth_min;
                    if (depth_delta > skip_interpolation_threshold)
                    {
                        continue;
                    }
                }

                dst_data[i] = (uint32_t)(neighbors[0] * lut_data[i].weight[0] + neighbors[1] * lut_data[i].weight[1] +
                                         neighbors[2] * lut_data[i].weight[2] + neighbors[3] * lut_data[i].weight[3] +
                                         0.5f);
            }
            else
            {
                printf("Unexpected interpolation type!\n");
                exit(-1);
            }
        }
    }
}


k4a_image_t downscale_image_2x2_binning(const k4a_image_t color_image)
{
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
    int color_image_downscaled_width_pixels = color_image_width_pixels / 2;
    int color_image_downscaled_height_pixels = color_image_height_pixels / 2;
    k4a_image_t color_image_downscaled = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                                 color_image_downscaled_width_pixels,
                                                 color_image_downscaled_height_pixels,
                                                 color_image_downscaled_width_pixels * 4 * (int)sizeof(uint8_t),
                                                 &color_image_downscaled))
    {
        printf("Failed to create downscaled color image\n");
        return color_image_downscaled;
    }

    uint8_t *color_image_data = k4a_image_get_buffer(color_image);
    uint8_t *color_image_downscaled_data = k4a_image_get_buffer(color_image_downscaled);
    for (int j = 0; j < color_image_downscaled_height_pixels; j++)
    {
        for (int i = 0; i < color_image_downscaled_width_pixels; i++)
        {
            int index_downscaled = j * color_image_downscaled_width_pixels + i;
            int index_tl = (j * 2 + 0) * color_image_width_pixels + i * 2 + 0;
            int index_tr = (j * 2 + 0) * color_image_width_pixels + i * 2 + 1;
            int index_bl = (j * 2 + 1) * color_image_width_pixels + i * 2 + 0;
            int index_br = (j * 2 + 1) * color_image_width_pixels + i * 2 + 1;

            color_image_downscaled_data[4 * index_downscaled + 0] = (uint8_t)(
                (color_image_data[4 * index_tl + 0] + color_image_data[4 * index_tr + 0] +
                 color_image_data[4 * index_bl + 0] + color_image_data[4 * index_br + 0]) /
                4.0f);
            color_image_downscaled_data[4 * index_downscaled + 1] = (uint8_t)(
                (color_image_data[4 * index_tl + 1] + color_image_data[4 * index_tr + 1] +
                 color_image_data[4 * index_bl + 1] + color_image_data[4 * index_br + 1]) /
                4.0f);
            color_image_downscaled_data[4 * index_downscaled + 2] = (uint8_t)(
                (color_image_data[4 * index_tl + 2] + color_image_data[4 * index_tr + 2] +
                 color_image_data[4 * index_bl + 2] + color_image_data[4 * index_br + 2]) /
                4.0f);
            color_image_downscaled_data[4 * index_downscaled + 3] = (uint8_t)(
                (color_image_data[4 * index_tl + 3] + color_image_data[4 * index_tr + 3] +
                 color_image_data[4 * index_bl + 3] + color_image_data[4 * index_br + 3]) /
                4.0f);
        }
    }

    return color_image_downscaled;
}


bool create_folder(const std::string& folderpath)
{
    // Create output folders
    if (!std::experimental::filesystem::is_directory(folderpath) || !std::experimental::filesystem::exists(folderpath))
    {
        return std::experimental::filesystem::create_directory(folderpath);
    }
    
    return false;
}


void getCamMatDistCoeffs(k4a_calibration_t& calib, const k4a_calibration_type_t& calib_type, cv::Mat& cam_mat, cv::Mat& dist_coeffs)
{

    if (calib_type == K4A_CALIBRATION_TYPE_COLOR)
    {
        k4a_calibration_intrinsic_parameters_t *color_intrinsics = &calib.color_camera_calibration.intrinsics.parameters;
        //std::vector<float> _color_camera_matrix = {1.f, 0.f, 960.f, 0.f, 1.f, 540.f, 0.f, 0.f, 1.f};
        std::vector<float> _color_camera_matrix = {color_intrinsics->param.fx, 0.f, color_intrinsics->param.cx,
                                                   0.f, color_intrinsics->param.fy, color_intrinsics->param.cy,
                                                   0.f, 0.f, 1.f
                                                  };
        cam_mat = cv::Mat(3, 3, CV_32F, &_color_camera_matrix[0]);

        //std::vector<float> _color_dist_coeffs = { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
        std::vector<float> _color_dist_coeffs = { color_intrinsics->param.k1, color_intrinsics->param.k2, color_intrinsics->param.p1,
                                                  color_intrinsics->param.p2, color_intrinsics->param.k3, color_intrinsics->param.k4,
                                                  color_intrinsics->param.k5, color_intrinsics->param.k6
                                                };
        dist_coeffs = cv::Mat(8, 1, CV_32F, &_color_dist_coeffs[0]);
    }
    else if (calib_type == K4A_CALIBRATION_TYPE_DEPTH)
    {
        k4a_calibration_intrinsic_parameters_t *intrinsics = &calib.depth_camera_calibration.intrinsics.parameters;
        std::vector<float> _camera_matrix = {1.f, 0.f, 960.f, 0.f, 1.f, 540.f, 0.f, 0.f, 1.f};
        //std::vector<float> _camera_matrix = {intrinsics->param.fx, 0.f, intrinsics->param.cx, 0.f, intrinsics->param.fy, intrinsics->param.cy, 0.f, 0.f, 1.f};
        cam_mat = cv::Mat(3, 3, CV_32F, &_camera_matrix[0]);

        std::vector<float> _dist_coeffs = { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
        //intrinsics->param.k1, intrinsics->param.k2, intrinsics->param.p1,
        //                                intrinsics->param.p2, intrinsics->param.k3, intrinsics->param.k4,
        //                                intrinsics->param.k5, intrinsics->param.k6};
        dist_coeffs = cv::Mat(8, 1, CV_32F, &_dist_coeffs[0]);
    }
}

struct color_point_t
{
    int16_t xyz[3];
    uint8_t rgb[3];
};
