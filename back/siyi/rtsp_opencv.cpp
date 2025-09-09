#include <opencv2/opencv.hpp>
#include <iostream>

int main() {

  // RTSP URL

  std::string rtsp_url = "rtsp://192.168.1.25:8554/main.264";

  // 打开RTSP流

  cv::VideoCapture cap(rtsp_url);

  if (!cap.isOpened()) {

    std::cerr << "Error: Unable to open the RTSP stream" << std::endl;

    return -1;

  }

  // 获取视频帧率

  double fps = cap.get(cv::CAP_PROP_FPS);

  std::cout << "FPS: " << fps << std::endl;

  cv::Mat frame;

  while (true) {

    // 读取视频帧

    if (!cap.read(frame)) {

      std::cerr << "Error: Unable to read frame from RTSP stream" << std::endl;

      break;

    }

    // 显示视频帧

    cv::imshow("RTSP Stream", frame);

    // 按 'q' 键退出

    if (cv::waitKey(1) == 'q') {

      break;

    }

  }

  // 释放资源

  cap.release();

  cv::destroyAllWindows();

  return 0;

}