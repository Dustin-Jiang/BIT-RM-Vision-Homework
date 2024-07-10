#include <opencv2/opencv.hpp>
#include <iostream>

#include "pipeline.hpp"

#ifndef VIDEOPLAY_CPP
#define VIDEOPLAY_CPP

class Player
{
private:
  cv::VideoCapture cap;
  cv::Mat frame;

public:
  bool play_end;
  bool playing = false;

  Player(const std::string &filename) : cap(filename), play_end(true)
  {
    if (!cap.isOpened())
    {
      std::cerr << "Error opening video file or file not found!" << std::endl;
      play_end = false;
      return;
    }

    cv::namedWindow("Video", cv::WINDOW_AUTOSIZE);
  }

  bool isOpened()
  {
    return cap.isOpened();
  }

  bool next()
  {
    bool success = cap.read(frame);
    if (!success)
    {
      std::cout << "Can't receive frame (stream end?)." << std::endl;
      play_end = false;
    }

    return success;
  }

  template <typename Tp> Tp calc(pipeline::Task<Tp> p)
  {
    cv::Mat buff = frame;
    return p(buff);
  }

  cv::Mat calc(pipeline::DispPipeline p)
  {
    cv::Mat buff = frame;
    for (auto &task : p)
    {
      buff = task(buff);
    }
    return buff;
  }

  void show(pipeline::DispPipeline p)
  {
    cv::Mat buff = frame;
    for (auto &task : p)
    {
      buff = task(buff);
    }
    cv::imshow("Video", buff);
  }

  void show()
  {
    cv::imshow("Video", frame);
  }

  void stop()
  {
    play_end = false;
  }

  bool save_frame()
  {
    return cv::imwrite("frame.jpg", frame);
  }

  bool save_frame(const std::string &filename)
  {
    return cv::imwrite(filename, frame);
  }

  ~Player()
  {
    cap.release();
    cv::destroyAllWindows();
  }
};


#endif