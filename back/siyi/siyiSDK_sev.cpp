#include <iostream>

#include <cstring>

#include <cerrno>

#include <sys/socket.h>

#include <netinet/in.h>

#include <arpa/inet.h>

#include <unistd.h>

#define RECV_BUUF_SIZE 64

#define SERVER_PORT 37260 // 云台相机（服务端）端口号

#define SERVER_IP "192.168.255.25" // 云台相机（服务端）IP

int main(int argc, char *argv[])

{

  int sockfd;

  int ret, recv_len;

  struct sockaddr_in send_addr, recv_addr;

  unsigned char send_buf[] = 

  {0x55, 0x66, 0x01, 0x02, 0x00, 0x00, 0x00, 0x07, 0x64, 0x64,0x3d,0xcf}; // 对应功能的帧协议,十六进制数据

  unsigned char recv_buf[RECV_BUUF_SIZE] = {0};

  // 创建 UDP 套接字

  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {

    perror("socket");

    return 1;

  }

  // 设置云台相机的 ip 和端口号

  memset(&send_addr, 0, sizeof(send_addr));

  send_addr.sin_family = AF_INET;

  send_addr.sin_addr.s_addr = inet_addr(SERVER_IP);

  send_addr.sin_port = htons(SERVER_PORT);

  // 发送帧数据

  std::cout << "Send HEX data" << std::endl;

  socklen_t addr_len = sizeof(struct sockaddr_in);

  if (sendto(sockfd, send_buf, sizeof(send_buf), 0, (struct sockaddr *)&send_addr, addr_len) < 0) {

    perror("sendto");

    return 1;

  }

  // 接收云台相机的返回数据

  recv_len = recvfrom(sockfd, recv_buf, RECV_BUUF_SIZE, 0, (struct sockaddr *)&recv_addr, &addr_len);

  if (recv_len < 0) {

    perror("recvfrom");

    return 1;

  }

  // 十六进制形式打印接收到的数据

  std::cout << "Received HEX data: ";

  for (int i = 0; i < recv_len; i++) {

    std::cout << std::hex << static_cast<int>(recv_buf[i]) << " ";

  }

  std::cout << std::endl;

  close(sockfd); // 关闭套接字

  return 0;

}