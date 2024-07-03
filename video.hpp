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
  bool playing;

  Player(const std::string &filename) : cap(filename), playing(true)
  {
    if (!cap.isOpened())
    {
      std::cerr << "Error opening video file or file not found!" << std::endl;
      playing = false;
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
      playing = false;
    }

    return success;
  }

  void show(pipeline::Pipeline p)
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
    playing = false;
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