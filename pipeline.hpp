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
}

#endif