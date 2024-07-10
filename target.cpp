#include <iostream>
#include <opencv2/opencv.hpp>

#include "video.hpp"

int main()
{
  // 打开视频文件
  Player player{"target.mp4"};

  bool success = player.isOpened() && player.next();
  double scale = 1.0;

  // 缩放
  auto scale_img = [&](cv::Mat &frame) { return pipeline::scale(frame, scale); };

  while (player.play_end)
  {
    // 如果读取失败，则退出循环
    if (!success)
    {
      std::cerr << "Can't receive frame (stream end?). Exiting ..." << std::endl;
      player.stop();
      break;
    }

    // 计算帧数据
    pipeline::Task<std::vector<std::vector<cv::Point2f>>> calcs = [&](cv::Mat &frame)
    {
      cv::Mat buff;
      frame.copyTo(buff);
      buff = pipeline::binary(buff);
      return pipeline::detect(buff);
    };
    
    auto rects = player.calc(calcs);

    // 渲染帧
    auto render = [&](cv::Mat &frame)
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
    };

    // 显示帧
    player.show({ render, scale_img });

    // 播放器逻辑
    if (player.playing)
    {
      success = player.next();
    }

    char key = cv::waitKey(15);
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
    if (key == ' ')
    {
      player.playing = !player.playing;
    }
  }

  return 0;
}