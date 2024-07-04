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

  pipeline::DispPipeline disps;

  auto binary = ([&](cv::Mat &frame) {
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

  auto detect = ([&](cv::Mat &frame) -> std::vector<std::vector<cv::Point2f>>
  {
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

      float max_width = 0.0;
      for (int i = 0; i < 4; i++)
      {
        float delta_x = vertices[i].x - vertices[(i + 1) % 4].x;
        if (delta_x > max_width)
        {
          max_width = delta_x;
        }
      }

      if (max_height < 20.0) continue;
      if (max_width > 20.0) continue;

      rects.push_back(rect);
    }

    for (auto rect : rects)
    {
      cv::Point2f verts[4];
      rect.points(verts);

      // cv::line(buff, verts[0], verts[1], cv::Scalar(0, 255, 0), 2);
      // cv::line(buff, verts[1], verts[2], cv::Scalar(0, 255, 0), 2);
      // cv::line(buff, verts[2], verts[3], cv::Scalar(0, 255, 0), 2);
      // cv::line(buff, verts[3], verts[0], cv::Scalar(0, 255, 0), 2);
    }

    std::vector<std::vector<cv::RotatedRect>> pairs;

    for (int i = 0; i < rects.size(); i++)
    {
      for (int j = i + 1; j < rects.size(); j++)
      {
        auto rect1 = rects[i];
        auto rect2 = rects[j];

        if (
          std::fabs(rect1.center.y - rect2.center.y) > 15.0 ||
          std::fabs(rect1.center.x - rect2.center.x) > 200.0
        )
        {
          continue;
        }

        cv::Point2f vert1[4], vert2[4];
        rect1.points(vert1);
        rect2.points(vert2);

        std::sort(vert1, vert1 + 4, [](cv::Point2f a, cv::Point2f b) {
          return a.y < b.y;
        });
        std::sort(vert2, vert2 + 4, [](cv::Point2f a, cv::Point2f b) {
          return a.y < b.y;
        });

        auto k1 = (vert1[0].x + vert1[1].x - vert1[2].x - vert1[3].x) / (vert1[0].y + vert1[1].y - vert1[2].y - vert1[3].y);
        auto k2 = (vert2[0].x + vert2[1].x - vert2[2].x - vert2[3].x) / (vert2[0].y + vert2[1].y - vert2[2].y - vert2[3].y);

        // 判断斜率是否相近 去除明显不平行
        if (k1 * k2 <= 0.0 && std::fabs(k1 - k2) > 0.1)
        {
          continue;
        }

        pairs.push_back({rect1, rect2});
      }
    }

    std::vector<std::vector<cv::Point2f>> results;
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

        auto top = (vertices[0] + vertices[1]) / 2;
        auto bottom = (vertices[2] + vertices[3]) / 2;

        auto center = (top + bottom) / 2;

        verts.push_back(center + (top - center) * 2);
        verts.push_back(center + (bottom - center) * 2);
      }

      results.push_back(verts);
    }
    
    return results;
  });

  // 缩放
  auto scale_img = [&](cv::Mat &frame) { return pipeline::scale(frame, scale); };

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
    pipeline::Task<std::vector<std::vector<cv::Point2f>>> calcs = [&](cv::Mat &frame)
    {
      cv::Mat buff;
      frame.copyTo(buff);
      buff = binary(buff);
      return detect(buff);
    };
    
    auto rects = player.calc(calcs);

    player.show({ [&](cv::Mat &frame)
    {
      cv::Mat buff;
      frame.copyTo(buff);

      for (auto verts : rects)
      {
        cv::line(buff, verts[0], verts[1], cv::Scalar(0, 0, 255), 2);
        cv::line(buff, verts[1], verts[3], cv::Scalar(0, 0, 255), 2);
        cv::line(buff, verts[2], verts[3], cv::Scalar(0, 0, 255), 2);
        cv::line(buff, verts[2], verts[0], cv::Scalar(0, 0, 255), 2);
      }

      return buff;
    }, scale_img });

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