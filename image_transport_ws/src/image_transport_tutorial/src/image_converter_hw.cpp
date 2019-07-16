#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>

cv::Mat MulImg_SingleWin(const cv::Mat &img, const cv::Mat &gray_img, cv::Mat &bin_img)
{
  cv::Mat tmp_g, tmp_b, base;
  cv::cvtColor(gray_img, tmp_g, CV_GRAY2BGR);
  cv::cvtColor(bin_img, tmp_b, CV_GRAY2BGR);

  int width = img.cols + tmp_g.cols;
  int height = img.rows + tmp_g.rows;

  base = cv::Mat::zeros(height, width, img.type());
  cv::Rect ROI = cv::Rect(0, 0, img.cols, img.rows);
  img.copyTo(base(ROI));

  ROI.x = img.cols;
  tmp_g.copyTo(base(ROI));

  ROI.x = 0;
  ROI.y = img.rows;
  tmp_b.copyTo(base(ROI));

  cv::putText(base, "Original Image", cv::Point(base.cols / 4 - 100, base.rows / 2 - 50), 0, 1.5, cv::Scalar(0, 200, 255), 4, CV_AA);
  cv::putText(base, "Gray Image", cv::Point(3 * base.cols / 4 - 100, base.rows / 2 - 50), 0, 1.5, cv::Scalar(0, 200, 255), 4, CV_AA);
  cv::putText(base, "Binary Image", cv::Point(base.cols / 4 - 100, base.rows - 50), 0, 1.5, cv::Scalar(0, 200, 255), 4, CV_AA);
  cv::putText(base, "Caputure the image", cv::Point(5 * base.cols / 8, 3 * base.rows / 4), 0, 1.0, cv::Scalar(200, 200, 200), 4, CV_AA);
  cv::putText(base, "by pressing the 's' key", cv::Point(5 * base.cols / 8 - 20, 3 * base.rows / 4 + 50), 0, 1.0, cv::Scalar(200, 200, 200), 4, CV_AA);

  cv::Mat show_img;
  cv::resize(base, show_img, cv::Size(), 0.6, 0.6);
  cv::imshow("Image Window", show_img);

  return base;
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_raw, image_pub_bin, image_pub_BB;

public:
  ImageConverter()
      : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);

    image_pub_raw = it_.advertise("/image_converter/output_raw", 1);
  }

  void imageCb(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat gray_img, bin_img, base;
    // bgr to gray
    cv::cvtColor(cv_ptr->image, gray_img, CV_BGR2GRAY);

    // gray to bin
    cv::threshold(gray_img, bin_img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    base = MulImg_SingleWin(cv_ptr->image, gray_img, bin_img);

    int key = cv::waitKey(1);
    if (key == 's')
    {
      // labeling
      cv::Mat label_id, stats, cent;
      int nLab = cv::connectedComponentsWithStats(bin_img, label_id, stats, cent, 8);

      // label_id -> color
      std::vector<cv::Vec3b> color(nLab);
      color[0] = cv::Vec3b(0, 0, 0);
      for (int i = 1; i < nLab; ++i)
        color[i] = cv::Vec3b((rand() & 255), (rand() & 255), (rand() & 255));

      // Show labeling image
      cv::Mat label_img(bin_img.size(), CV_8UC3);
      for (int j = 0; j < label_img.rows; ++j)
      {
        int *lb = label_id.ptr<int>(j);
        cv::Vec3b *pix = label_img.ptr<cv::Vec3b>(j);
        for (int i = 0; i < label_img.cols; ++i)
          pix[i] = color[lb[i]];
      }

      // Show BB image (Max Area)
      int max_area = -999;
      int max_areaID = 0;
      for (int i = 1; i < nLab; ++i)
      {
        int *lb = stats.ptr<int>(i);
        if (max_area < lb[4])
        {
          max_area = lb[4];
          max_areaID = i;
        }
      }
      int x = stats.ptr<int>(max_areaID)[0]; // begin point
      int y = stats.ptr<int>(max_areaID)[1]; // end point
      int w = stats.ptr<int>(max_areaID)[2]; // BB width
      int h = stats.ptr<int>(max_areaID)[3]; // BB height
      cv::rectangle(label_img, cv::Rect(x, y, w, h), cv::Scalar(0, 255, 0), 2);

      // Display Max_area num
      std::stringstream area_num;
      area_num << stats.ptr<int>(max_areaID)[4];
      cv::putText(label_img, "Area: " + area_num.str(), cv::Point(x+5, y+20), 0, 0.7, cv::Scalar(0, 255, 255), 2);

      cv::imshow("label_img", label_img);
      cv::waitKey();
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
