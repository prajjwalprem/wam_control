#include <cstdio>
#include "KinectDK.h"


crobot::KinectDK::KinectDK()
{
    _config                  = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    _config.depth_mode       = K4A_DEPTH_MODE_NFOV_UNBINNED;
    _config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    _config.camera_fps       = K4A_FRAMES_PER_SECOND_30;
    _config.color_format     = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    _config.synchronized_images_only = true;

    _xy_table                = nullptr;
    _point_cloud             = nullptr;
    _depth_image             = nullptr;
    _color_image             = nullptr;
    _capture                 = nullptr;
    _calibration             = k4a_calibration_t();
}

crobot::KinectDK::~KinectDK()
{
    k4a_image_release(_depth_image);
    k4a_capture_release(_capture);
    k4a_image_release(_xy_table);
    k4a_image_release(_point_cloud);
}


uint32_t crobot::KinectDK::device_count()
{
	return k4a_device_get_installed_count();
}


void crobot::KinectDK::create_xy_table(const k4a_calibration_t* calibration, k4a_image_t xy_table)
{
	auto* table_data = static_cast<k4a_float2_t*>(static_cast<void*>(k4a_image_get_buffer(xy_table)));

    const int width  = calibration->depth_camera_calibration.resolution_width;
    const int height = calibration->depth_camera_calibration.resolution_height;

    k4a_float2_t p;
    k4a_float3_t ray;
    int valid;

    for (int y = 0, idx = 0; y < height; y++)
    {
        p.xy.y = static_cast<float>(y);
        for (int x = 0; x < width; x++, idx++)
        {
            p.xy.x = static_cast<float>(x);

            k4a_calibration_2d_to_3d(
                calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

            if (valid)
            {
                table_data[idx].xy.x = ray.xyz.x;
                table_data[idx].xy.y = ray.xyz.y;
            }
            else
            {
                table_data[idx].xy.x = nanf("");
                table_data[idx].xy.y = nanf("");
            }
        }
    }
}


int crobot::KinectDK::connect(int device_index)
{
    if (K4A_RESULT_SUCCEEDED != k4a_device_open(device_index, &_device))
    {
        printf("%d: Failed to open _device\n", device_index);
    }

    size_t serial_number_length = 0;

    if (K4A_BUFFER_RESULT_TOO_SMALL != k4a_device_get_serialnum(_device, nullptr, &serial_number_length))
    {
        printf("%d: Failed to get serial number length\n", device_index);
        k4a_device_close(_device);
        _device = nullptr;
        return 1;
    }

    _serial_number = static_cast<char*>(malloc(serial_number_length));
    if (_serial_number == nullptr)
    {
        printf("%d: Failed to allocate memory for serial number (%zu bytes)\n", device_index, serial_number_length);
        k4a_device_close(_device);
        _device = nullptr;
        return 2;
    }

    if (K4A_BUFFER_RESULT_SUCCEEDED != k4a_device_get_serialnum(_device, _serial_number, &serial_number_length))
    {
        printf("%d: Failed to get serial number\n", device_index);
        free(_serial_number);
        _serial_number = nullptr;
        k4a_device_close(_device);
        _device = nullptr;
        return 3;
    }

    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(_device, _config.depth_mode, _config.color_resolution, &_calibration))
    {
        printf("Failed to get calibration\n");
        free(_serial_number);
        _serial_number = nullptr;
        k4a_device_close(_device);
        _device = nullptr;
        return 4;
    }

    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        _calibration.depth_camera_calibration.resolution_width,
        _calibration.depth_camera_calibration.resolution_height,
        _calibration.depth_camera_calibration.resolution_width * static_cast<int>(sizeof(k4a_float2_t)),
        &_xy_table);

    create_xy_table(&_calibration, _xy_table);

    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        _calibration.depth_camera_calibration.resolution_width,
        _calibration.depth_camera_calibration.resolution_height,
        _calibration.depth_camera_calibration.resolution_width * static_cast<int>(sizeof(k4a_float3_t)),
        &_point_cloud);

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(_device, &_config))
    {
        printf("Failed to start cameras\n");
        disconnect();
    }

    return 0;
}

int crobot::KinectDK::connect_default()
{
    return connect(K4A_DEVICE_DEFAULT);
}

int crobot::KinectDK::disconnect()
{
	if (_device != nullptr)
	{
        free(_serial_number);
        _serial_number = nullptr;
		k4a_device_close(_device);
        _device = nullptr;
	}

	return 0;
}


int crobot::KinectDK::get_capture()
{
    // Get a capture
    const int timeout_in_ms = 1000;
    switch (k4a_device_get_capture(_device, &_capture, timeout_in_ms))
    {
    case K4A_WAIT_RESULT_SUCCEEDED:
        break;
    case K4A_WAIT_RESULT_TIMEOUT:
        printf("Timed out waiting for a capture\n");
        disconnect();
        return 1;
    case K4A_WAIT_RESULT_FAILED:
        printf("Failed to read a capture\n");
        return 1;
    }

    return 0;
}


int crobot::KinectDK::get_depth_image_from_last_capture(cv::Mat& mat) const
{
    // Access the Color 8UC4 image
    const k4a_image_t depth_image = k4a_capture_get_depth_image(_capture);
    if (depth_image == nullptr)
    {
        printf("depth image not available from current capture.\n");
        return 1;
    }

    // get raw buffer
    uint8_t* buffer = k4a_image_get_buffer(depth_image);

    // convert the raw buffer to cv::Mat
    const int rows = k4a_image_get_height_pixels(depth_image);
    const int cols = k4a_image_get_width_pixels(depth_image);
    cv::Mat(rows, cols, CV_16UC1, static_cast<void*>(buffer), cv::Mat::AUTO_STEP).copyTo(mat);

    // Release the image
    k4a_image_release(depth_image);
    return 0;
}

int crobot::KinectDK::get_color_image_from_last_capture(cv::Mat& mat) const
{
    // Access the Color 8UC4 image
    const k4a_image_t color_image = k4a_capture_get_color_image(_capture);
    if (color_image == nullptr)
    {
        printf("color image not available from current capture.\n");
        return 1;
    }

    // get raw buffer
    uint8_t* buffer = k4a_image_get_buffer(color_image);

    // convert the raw buffer to cv::Mat
    const int rows = k4a_image_get_height_pixels(color_image);
    const int cols = k4a_image_get_width_pixels(color_image);
    cv::Mat(rows, cols, CV_8UC4, static_cast<void*>(buffer), cv::Mat::AUTO_STEP).copyTo(mat);

    // Release the image
    k4a_image_release(color_image);
    return 0;

}

int crobot::KinectDK::get_rgbd_image_from_last_capture(cv::Mat& mat) const {
    const k4a_image_t depth_image = k4a_capture_get_depth_image(_capture);
    if (depth_image == nullptr)
    {
        printf("depth image not available from current capture.\n");
        return 1;
    }

    const k4a_image_t color_image = k4a_capture_get_color_image(_capture);
    if (color_image == nullptr)
    {
        printf("color image not available from current capture.\n");
        return 1;
    }

    auto transformation = k4a_transformation_create(&_calibration);
    k4a_image_t transformed_image = NULL;

    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);

    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t),
        &transformed_image))
    {
        printf("Failed to create transformed depth image\n");
        return false;
    }


    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera(transformation, depth_image, transformed_image))
    {
        std::cout << "Trans failed!" << std::endl;
    }

    uint8_t* buffer = k4a_image_get_buffer(transformed_image); 
    const int rows  = k4a_image_get_height_pixels(transformed_image);
    const int cols  = k4a_image_get_width_pixels(transformed_image);
    cv::Mat(rows, cols, CV_16UC1, static_cast<void*>(buffer), cv::Mat::AUTO_STEP).copyTo(mat);

    return 0; 
}

int crobot::KinectDK::release_capture() const
{
    k4a_capture_release(_capture);
    return 0;
}


int crobot::KinectDK::helper() const
{
    // Access the depth16 image
    k4a_image_t image = k4a_capture_get_depth_image(_capture);
    if (image != NULL)
    {
        printf(" | Depth16 res:%4dx%4d stride:%5d\n",
            k4a_image_get_height_pixels(image),
            k4a_image_get_width_pixels(image),
            k4a_image_get_stride_bytes(image));

        // Release the image
        k4a_image_release(image);
    }

    return 0;
}
