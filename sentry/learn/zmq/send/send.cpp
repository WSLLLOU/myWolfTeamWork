//
//  Weather update server in C++
//  Binds PUB socket to tcp://*:5556
//  Publishes random weather updates
//
#include <iostream>
#include <zmq.hpp>

// 套接字内容
struct Msgs{    
    int id;
    int information;
    Msgs(int id, int infor) : id(id), information(infor) {}
};

int main () {
    //  Prepare our context and publisher
    zmq::context_t  context(1);
    zmq::socket_t   publisher(context, zmq::socket_type::pub);
    publisher.bind("tcp://*:5556");

    // fake date
    Msgs pc1(0, 0);

    int i=0;
    while (true) {
        pc1.information = ++i;
        zmq::message_t send_message(sizeof(pc1));
        memcpy(send_message.data(),&pc1,sizeof(pc1));
        publisher.send(send_message);
        // 
        std::cout << i << std::endl;
    }
    return 0;
}