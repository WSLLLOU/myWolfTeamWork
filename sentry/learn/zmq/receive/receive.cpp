#include <iostream>
#include <zmq.hpp>
#include <unistd.h>

// 套接字内容
struct Msgs{    
    int id;
    int information;
    Msgs(int id, int infor) : id(id), information(infor) {}
};


int main (int argc, char *argv[])
{
    zmq::context_t context (1);

    //  Socket to talk to server
    std::cout << "waiting server...\n" << std::endl;
    zmq::socket_t subscriber (context, ZMQ_SUB);
    subscriber.connect("tcp://localhost:5556");
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    Msgs pc2 = Msgs(2, 0);
    while (true)
    {
        usleep(10000);    // 单位是微秒 1000000us = 1s
        zmq::message_t recv_message(sizeof(Msgs));
        bool receive = false;
        while (subscriber.recv(&recv_message , ZMQ_DONTWAIT)) { // 此处 while 是为了获取到最新的msgs, 确保消息实时性; 使用 if 会导致只获取到一次消息队列之前的信息, 导致信息滞后越来越严重
            memcpy(&pc2, recv_message.data(), sizeof(pc2));
            receive = true;
        }
        std::cout << "收到信息  "<< receive <<std::endl;

        std::cout << pc2.information << std::endl;
    }
    
    return 0;
}