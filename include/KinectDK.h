#ifndef CROBOT_KINECT_DK_H
#define CROBOT_KINECT_DK_H

#include <opencv2/opencv.hpp>
#include <k4a/k4a.h>


namespace crobot
{
	class KinectDK
	{
	public:
		k4a_device_t               _device = nullptr;
		char*                      _serial_number = nullptr;
		k4a_device_configuration_t _config;
		k4a_image_t                _xy_table;
		k4a_image_t                _point_cloud;
		k4a_capture_t              _capture;
		k4a_image_t                _depth_image;
		k4a_image_t                _color_image;
		k4a_calibration_t          _calibration;

	public:
		KinectDK();
		virtual ~KinectDK();

		static uint32_t device_count();
		static void create_xy_table(const k4a_calibration_t* calibration, k4a_image_t xy_table);
		int connect(int device_index);
		int connect_default();
		int disconnect();

		int get_capture();
		int get_depth_image_from_last_capture(cv::Mat& depth_image) const;
		int get_color_image_from_last_capture(cv::Mat& color_image) const;
		int get_rgbd_image_from_last_capture(cv::Mat& rgbd_image) const;
		int release_capture() const;
	
		int helper() const;
	};

}

#endif /* CROBOT_KINECT_DK_H */
