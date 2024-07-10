# 程序说明

- `w` 单帧步进
- `a` 缩小
- `s` 放大
- `SPACE` 播放

效果样例在根目录下 `result.mp4` 中. 

## 代码简述

`video.hpp` 下实现了 `Player` 类封装视频播放, 通过 `Player.calc()` 传入 lambda 以对当前帧进行计算, 通过 `Player.show()` 传入 lambda vector 以对当前帧进行一定处理后显示. 

`pipeline.hpp` 中 `Pipeline` 命名空间下定义了常用的处理函数, 和比较复杂的处理逻辑. 

`target.cpp` 为主程序, 进行了具体实现.

## 编译

在根目录下编译 `target.cpp` 即可得到执行文件 `target`. 

## 尚未实现

- [ ] 大津法显示数字
- [ ] 模板识别数字
- [ ] 在结果上显示文字 ~~(编译 OpenCV 的时候没带上 Qt 库导致的)~~