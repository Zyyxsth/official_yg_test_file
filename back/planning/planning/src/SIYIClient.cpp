#include "SIYIClient/SIYIClient.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <vector>
#include <cmath>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 弧度转度数
inline double rad2deg(double radians) {
    return radians * (180.0 / M_PI);
}

// 计算向量与主方向的角度
double vectorAngle(const Vec2f& unitVector, const Vec2f& mainDirection) {
    float dotProduct = unitVector[0] * mainDirection[0] + unitVector[1] * mainDirection[1];
    float magnitudeUnitVector = norm(unitVector);
    if (magnitudeUnitVector == 0) return 0;
    double angleRad = acos(dotProduct / magnitudeUnitVector); // 结果是弧度
    return rad2deg(angleRad); // 转换为度数
}

SIYIClient::SIYIClient() {
    // 创建 UDP 套接字
    if ((sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket");
        throw std::runtime_error("Failed to create socket");
    }

    // 设置云台相机的 ip 和端口号
    memset(&send_addr_, 0, sizeof(send_addr_));
    send_addr_.sin_family = AF_INET;
    send_addr_.sin_addr.s_addr = inet_addr(SERVER_IP);
    send_addr_.sin_port = htons(SERVER_PORT);
}

SIYIClient::~SIYIClient() {
    close(sockfd_); // 关闭套接字
}

bool SIYIClient::sendReceive(const vector<unsigned char>& sendBuf, vector<unsigned char>& recvBuf) {
    cout << "Send HEX data" << endl;

    socklen_t addr_len = sizeof(struct sockaddr_in);

    // 确保接收缓冲区足够大
    recvBuf.resize(RECV_BUF_SIZE);

    // 使用一个新的 sockaddr_in 结构来接收发送方的地址信息
    struct sockaddr_in recv_addr;
    memset(&recv_addr, 0, sizeof(recv_addr));
    // socklen_t addr_len = sizeof(recv_addr);

    // 发送数据
    if (sendto(sockfd_, sendBuf.data(), sendBuf.size(), 0, (struct sockaddr *)&send_addr_, addr_len) < 0) {
        perror("sendto");
        return false;
    }

    int recv_len = recvfrom(sockfd_, recvBuf.data(), RECV_BUF_SIZE, 0, (struct sockaddr *)&send_addr_, &addr_len);

    if (recv_len < 0) {
        perror("recvfrom");
        return false;
    }

    // 十六进制形式打印接收到的数据
    cout << "Received HEX data: ";
    for (int i = 0; i < recv_len; i++) {
        cout << hex << static_cast<int>(recvBuf[i]) << " ";
    }
    cout << endl;

    return true;
}

bool SIYIClient::captureAndSaveFrame(const string& rtspUrl) {
    VideoCapture cap(rtspUrl);
    if (!cap.isOpened()) {
        cerr << "Error: Unable to open the RTSP stream" << endl;
        return false;
    }

    Mat frame;
    if (!cap.read(frame)) {
        cerr << "Error: Unable to read frame from RTSP stream" << endl;
        cap.release();
        return false;
    }

    string filename = "save_image.jpg"; // 确保文件名有扩展名
    bool isSaved = imwrite(filename, frame);
    if (!isSaved) {
        cerr << "Error: Unable to save the frame as an image" << endl;
        return false;
    }

    cout << "Frame saved as " << filename << endl;
    cap.release();
    return true;
}

Mat SIYIClient::captureFrame(const string& rtspUrl) {
    std::cout << rtspUrl << std::endl;
    VideoCapture cap(rtspUrl);
    std::cout << "capture true  :" << std::endl;
    if (!cap.isOpened()) {
        cerr << "Error: Unable to open the RTSP stream" << endl;
        return Mat();
    }

    Mat frame;
    if (!cap.read(frame)) {
        cerr << "Error: Unable to read frame from RTSP stream" << endl;
        cap.release();
        return Mat();
    }
    std::cout << "read problem !! :" << std::endl;
    // cv::imshow("RTSP Stream", frame);
    cap.release();
    std::cout << "release problem !! " << std::endl;
    return frame;
}

vector<Vec2f> SIYIClient::DetectDir(const cv::Mat& img_1, const cv::Mat& img_2) {
    cout << "detect--debug" << 0 << ", " << 1 << "???????? begin" << endl;

    if (img_1.empty() || img_2.empty()) {
        cerr << "Error: Unable to load one or both images." << endl;
    }
    
    // 旋转图像
    // Mat rotated_image_1;
    // rotate(img_1, rotated_image_1, ROTATE_90_COUNTERCLOCKWISE);

    // Mat rotated_image_2;
    // rotate(img_2, rotated_image_2, ROTATE_90_COUNTERCLOCKWISE);
    // cout << "detect--debug" << 0 << ", " << 2 << "???????? 旋转图像" << endl;


    // 缩放图像
    // Mat img1;
    // resize(rotated_image_1, img1, Size(), 0.4, 0.4, INTER_LINEAR);

    // Mat img2;
    // resize(rotated_image_2, img2, Size(), 0.4, 0.4, INTER_LINEAR);
    // cout << "detect--debug" << 0 << ", " << 2 << "???????? 缩放图像" << endl;


    // 初始化ORB检测器
    Ptr<ORB> orb = ORB::create();
    cout << "detect--debug" << 1 << ", " << 0 << "???????? 初始化ORB" << endl;
    vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    orb->detectAndCompute(img_1, noArray(), keypoints_1, descriptors_1);
    orb->detectAndCompute(img_2, noArray(), keypoints_2, descriptors_2);
    cout << "detect--debug" << 1 << ", " << 1 << "???????? orb->detectAndCompute" << endl;

    // 创建BFMatcher对象
    BFMatcher matcher(NORM_HAMMING, true);

    vector<DMatch> matches;
    matcher.match(descriptors_1, descriptors_2, matches);

    // 根据匹配对提取坐标
    vector<Point2f> img1_matches, img2_matches;
    for (const auto& match : matches) {
        img1_matches.push_back(keypoints_1[match.queryIdx].pt);
        img2_matches.push_back(keypoints_2[match.trainIdx].pt);
    }

    // 计算向量和主方向
    vector<Vec2f> vectors, unit_vectors, filtered_unit_vectors;
    for (size_t i = 0; i < img1_matches.size(); ++i) {
        vectors.push_back(Vec2f(img1_matches[i].x - img2_matches[i].x, img1_matches[i].y - img2_matches[i].y));
    }
    cout << "detect--debug" << 2 << ", " << 2 << "???????? 计算向量和主方向" << endl;

    for (const auto& vector : vectors) {
        float magnitude = norm(vector);
        unit_vectors.push_back(Vec2f(vector[0] / magnitude, vector[1] / magnitude));
    }

    float sum_x = 0, sum_y = 0;
    for (const auto& unit_vector : unit_vectors) {
        sum_x += unit_vector[0];
        sum_y += unit_vector[1];
    }

    float magnitude = sqrt(sum_x * sum_x + sum_y * sum_y);
    Vec2f main_direction(0, 0);
    if (magnitude > 0) {
        main_direction = Vec2f(sum_x / magnitude, sum_y / magnitude);
    }

    cout << "主方向：(" << main_direction[0] << ", " << main_direction[1] << ")" << endl;

    // 计算所有单位向量与主方向的角度
    vector<double> angles;
    for (const auto& unit_vector : unit_vectors) {
        angles.push_back(vectorAngle(unit_vector, main_direction));
    }

    // 排除差异最大的向量（例如，排除角度最大的25%）
    sort(angles.begin(), angles.end());
    double threshold_angle = angles[static_cast<size_t>(0.75 * angles.size())];

    for (size_t i = 0; i < angles.size(); ++i) {
        if (angles[i] < threshold_angle) {
            filtered_unit_vectors.push_back(unit_vectors[i]);
        }
    }

    // 重新计算主方向
    sum_x = 0, sum_y = 0;
    for (const auto& unit_vector : filtered_unit_vectors) {
        sum_x += unit_vector[0];
        sum_y += unit_vector[1];
    }
    magnitude = sqrt(sum_x * sum_x + sum_y * sum_y);
    if (magnitude > 0) {
        main_direction = Vec2f(sum_x / magnitude, sum_y / magnitude);
    }

    cout << "新的主方向：(" << main_direction[0] << ", " << main_direction[1] << ")" << endl;

}


void SIYIClient::Contrl(const std::vector<float>& main_direction) {
        if (main_direction.size() < 2) {
            std::cerr << "Error: main_direction must contain at least 2 elements." << std::endl;
            return;
        }

        int yaw = static_cast<int>(main_direction[0]);
        int pitch = static_cast<int>(main_direction[1]);

        // 构建控制云台的命令
        std::vector<unsigned char> cmd;
        cmd.push_back(0x55); // 起始字节
        cmd.push_back(0x66); // 命令字节
        cmd.push_back(0x01); // 控制命令
        cmd.push_back(0x04); // 数据长度
        cmd.push_back(0x00);
        cmd.push_back(0x00);
        cmd.push_back(0x00);
        cmd.push_back(0x0e); // 命令
        // cmd.push_back(static_cast<unsigned char>(yaw & 0xFF)); // yaw低字节
        // cmd.push_back(static_cast<unsigned char>((yaw >> 8) & 0xFF)); // yaw高字节
        // cmd.push_back(static_cast<unsigned char>(pitch & 0xFF)); // pitch低字节
        // cmd.push_back(static_cast<unsigned char>((pitch >> 8) & 0xFF)); // pitch高字节
        // 对于大端序，先添加高字节，然后添加低字节
        cmd.push_back(static_cast<unsigned char>((yaw >> 8) & 0xFF)); // yaw高字节
        cmd.push_back(static_cast<unsigned char>(yaw & 0xFF)); // yaw低字节
        cmd.push_back(static_cast<unsigned char>((pitch >> 8) & 0xFF)); // pitch高字节
        cmd.push_back(static_cast<unsigned char>(pitch & 0xFF)); // pitch低字节

        // 计算CRC16校验码
        uint16_t crc = CRC16_cal(cmd.data(), cmd.size(), 0);    
        cmd.push_back(static_cast<unsigned char>(crc & 0xFF)); // CRC低字节
        cmd.push_back(static_cast<unsigned char>((crc >> 8) & 0xFF)); // CRC高字节


        // 打印cmd向量
        std::cout << "Command vector (in hex): ";
        for (size_t i = 0; i < cmd.size(); ++i) {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(cmd[i]) << " ";
        }
        std::cout << std::endl;

        // 发送控制命令
        std::vector<unsigned char> recvBuf;
        if (!sendReceive(cmd, recvBuf)) {
            std::cerr << "Error: Failed to send control command." << std::endl;
        }


        // 请求相机姿态： 55 66 01 00 00 00 00 0d e8 05  ------------------------------------
        std::vector<unsigned char> cmd_test;
        cmd_test.push_back(0x55); // 起始字节
        cmd_test.push_back(0x66); // 命令字节
        cmd_test.push_back(0x01); // 控制命令
        cmd_test.push_back(0x00); // 数据长度
        cmd_test.push_back(0x00);
        cmd_test.push_back(0x00);
        cmd_test.push_back(0x00);
        cmd_test.push_back(0x0d); // 命令
        cmd_test.push_back(0xe8);
        cmd_test.push_back(0x05);
         // 打印cmd向量
        std::cout << "Command vector (in hex): ";
        for (size_t i = 0; i < cmd_test.size(); ++i) {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(cmd_test[i]) << " ";
        }
        std::cout << std::endl;

        // 发送控制命令
        std::vector<unsigned char> recvBuf_test;
        if (!sendReceive(cmd_test, recvBuf_test)) {
            std::cerr << "Error: Failed to send control command." << std::endl;
        }

        // -----------------------------------------------------------------

    }


uint16_t SIYIClient::CRC16_cal(uint8_t *ptr, uint32_t len, uint16_t crc_init) {
   uint16_t crc, oldcrc16;
uint8_t temp;
crc = crc_init;
while (len--!=0)
{
temp=(crc>>8)&0xff;
oldcrc16=crc16_tab[*ptr^temp];
crc=(crc<<8)^oldcrc16;
ptr++;
}
//crc=~crc; //??
return(crc);
}

// uint8_t SIYIClient::crc_check_16bites(uint8_t* pbuf, uint32_t len,uint32_t* p_result)
// {
// uint16_t crc_result = 0;
// crc_result= CRC16_cal(pbuf,len, 0);
// *p_result = crc_result;
// return 2;
// }

