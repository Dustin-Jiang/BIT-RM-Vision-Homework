#include <functional>
#include <vector>

#include <opencv2/opencv.hpp>

#ifndef PIPELINE_HPP
#define PIPELINE_HPP

namespace pipeline
{
  template <typename Tp> using Task = std::function<Tp(cv::Mat &)>;
  template <typename Tp> using Pipeline = std::vector<Task<Tp>>;
  using DispTask = Task<cv::Mat>;
  using DispPipeline = std::vector<DispTask>;

  cv::Mat grayscale(cv::Mat &frame)
  {
    cv::Mat grey;
    cv::cvtColor(frame, grey, cv::COLOR_BGR2GRAY);
    return grey;
  }

  cv::Mat channel_r(cv::Mat &frame)
  {
    cv::Mat filtered[3];
    cv::split(frame, filtered);

    return filtered[2];
  }

  cv::Mat channel_g(cv::Mat &frame)
  {
    cv::Mat filtered[3];
    cv::split(frame, filtered);

    return filtered[1];
  }

  cv::Mat channel_b(cv::Mat &frame)
  {
    cv::Mat filtered[3];
    cv::split(frame, filtered);

    return filtered[0];
  }

  cv::Mat scale(cv::Mat &frame, double factor)
  {
    cv::Mat scaled;
    cv::resize(frame, scaled, cv::Size(), factor, factor);
    return scaled;
  }

  cv::Mat generate_filter(cv::Mat &frame)
  {
    cv::Mat buff;
    
    cv::morphologyEx(
      frame, buff,
      cv::MORPH_OPEN,
      cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(8, 8)
      )
    );

    cv::bitwise_not(buff, buff);
    cv::threshold(buff, buff, 100.0, 255.0, cv::THRESH_BINARY);

    return buff;
  }

  auto binary = [](cv::Mat &frame)
  {
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
  };

  auto detect = [](cv::Mat &frame) -> std::vector<std::vector<cv::Point2f>>
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

        verts.push_back(center + (top - center) * 2.5);
        verts.push_back(center + (bottom - center) * 2.5);
      }

      results.push_back(verts);
    }
    
    return results;
  };
}

#endif