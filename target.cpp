#include <iostream>
#include <opencv2/opencv.hpp>

#include "video.hpp"

int main()
{
  // 打开视频文件
  Player player{"target.mp4"};
  bool success = player.isOpened() && player.next();
  double scale = 1.0;
  bool isBinary = false;
  bool isBorder = false;
  bool isDetect = false;

  pipeline::Pipeline operations;
  operations.push_back([&](cv::Mat &frame) {
    if (!isBinary) return frame;
    
    cv::Mat blue;
    frame.copyTo(blue);
    blue = pipeline::channel_b(blue);
    blue = pipeline::generate_filter(blue);

    cv::Mat green;
    frame.copyTo(green);
    green = pipeline::channel_g(green);
    green = pipeline::generate_filter(green);

    cv::Mat red;
    frame.copyTo(red);

    red = pipeline::channel_r(red);

    cv::bitwise_and(red, blue, red);
    cv::bitwise_and(red, green, red);

    cv::Mat buff;
    red.copyTo(buff);
    cv::threshold(buff, buff, 100.0, 255.0, cv::THRESH_BINARY);

    cv::morphologyEx(
      buff, buff,
      cv::MORPH_OPEN,
      cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(2, 2)
      ),
      cv::Point(-1, -1),
      4
    );

    return buff;
  });

  operations.push_back([&](cv::Mat &frame) {
    if (!isBorder) return frame;
    cv::Mat buff, res;
    frame.copyTo(buff);

    cv::Canny(buff, res, 100, 200);
    return res;
  });

  operations.push_back([&](cv::Mat &frame) {
    if (!isBinary) return frame;
    if (!isDetect) return frame;

    cv::Mat buff;
    frame.copyTo(buff);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(buff, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::sort(contours.begin(), contours.end(), [&](std::vector<cv::Point> a, std::vector<cv::Point> b) {
      return cv::contourArea(a) > cv::contourArea(b);
    });

    cv::cvtColor(buff, buff, cv::COLOR_GRAY2BGR);

    std::vector<cv::RotatedRect> rects;

    for (auto contour : contours)
    {
      double area = cv::contourArea(contour);
      if (area < 150) continue;
      if (area > 800) continue;

      cv::RotatedRect rect = cv::minAreaRect(contour);

      cv::Point2f vertices[4];
      rect.points(vertices);

      float max_height = 0.0;
      for (int i = 0; i < 4; i++)
      {
        float delta_y = vertices[i].y - vertices[(i + 1) % 4].y;
        if (delta_y > max_height)
        {
          max_height = delta_y;
        }
      }

      if (max_height < 20.0) continue;

      rects.push_back(rect);
    }

    std::vector<std::vector<cv::RotatedRect>> pairs;

    for (int i = 0; i < rects.size(); i++)
    {
      for (int j = i + 1; j < rects.size(); j++)
      {
        auto rect1 = rects[i];
        auto rect2 = rects[j];

        if (
          std::fabs(rect1.center.y - rect2.center.y) < 15.0 &&
          std::fabs(rect1.center.x - rect2.center.x) < 100.0
        )
        {
          pairs.push_back({rect1, rect2});
        }
      }
    }

    for (auto pair : pairs)
    {
      std::vector<cv::Point2f> verts;

      for (auto rect : pair)
      {
        cv::Point2f vertices[4];
        rect.points(vertices);

        std::sort(vertices, vertices + 4, [](cv::Point2f a, cv::Point2f b) {
          return a.y < b.y;
        });

        verts.push_back((vertices[0] + vertices[1]) / 2);
        verts.push_back((vertices[2] + vertices[3]) / 2);
      }

      cv::line(buff, verts[0], verts[1], cv::Scalar(0, 0, 255), 2);
      cv::line(buff, verts[1], verts[2], cv::Scalar(0, 0, 255), 2);
      cv::line(buff, verts[2], verts[3], cv::Scalar(0, 0, 255), 2);
      cv::line(buff, verts[3], verts[0], cv::Scalar(0, 0, 255), 2);
    }
    
    return buff;
  });

  // 缩放
  operations.push_back([&](cv::Mat &frame) { return pipeline::scale(frame, scale); });

  while (player.playing)
  {
    // 如果读取失败，则退出循环
    if (!success)
    {
      std::cerr << "Can't receive frame (stream end?). Exiting ..." << std::endl;
      player.stop();
      break;
    }

    // 显示帧
    player.show(operations);

    char key = cv::waitKey(0);
    if (key == 'q')
    {
      player.stop();
    }
    if (key == 'w')
    {
      success = player.next();
    }
    if (key == 'a')
    {
      scale = (scale <= 0.15) ? 0.1 : scale - 0.1;
    }
    if (key == 's')
    {
      if (!player.save_frame())
      {
        std::cerr << "Error saving frame!" << std::endl;
      }
    }
    if (key == 'd')
    {
      scale += 0.1;
    }
    if (key == 'e')
    {
      isBinary = !isBinary;
    }
    if (key == 'b')
    {
      isBorder = !isBorder;
    }
    if (key == 'c')
    {
      isDetect = !isDetect;
    }
  }

  return 0;
}