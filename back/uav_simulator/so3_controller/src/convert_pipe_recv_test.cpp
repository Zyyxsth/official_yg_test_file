#include <iostream>
/* message pipe */
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <string.h>

using namespace std;


/*################################*/
/* 直接运行可执行文件                */
/* ./convert_..._recv_test        */
/* 编译用：g++ convert_pipe_recv_test.cpp -o convert_pipe_recv_test -lrt  */
/*################################*/

/*#####################*/
/* Message struct      */
/* 消息类型列表：        */
/* 1：xyz速度，wxyz姿态  */
/*#####################*/
int msgid;
#define MSGKEY 1001
#define MSGTYPE 1
struct pipeMessageBuffer{
    long mType; // msg type
    double data[7]; // calocity xyz, quat wxyz
};


int main(int argc, char** argv){
    std::cout << "com in" << std::endl;
    // 获取消息队列
    int msqid = msgget(MSGKEY, 0666);
    if (msqid == -1) {
        perror("msgget failed");
        return 1;
    }

    while (true) {
        // 接收消息
        pipeMessageBuffer msg;
        if (msgrcv(msqid, &msg, sizeof(msg.data), MSGTYPE, 0) == -1) {
            perror("msgrcv failed");
            return 1;
        }

        std::cout << "Received message from queue." << std::endl;

        // 打印接收到的数据
        for (int i = 0; i < 7; i++) {
            std::cout << msg.data[i] << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}