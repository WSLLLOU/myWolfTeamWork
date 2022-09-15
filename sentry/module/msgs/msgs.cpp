#include "msgs.hpp"


ReceiveCarInfo Msgs::receiveCarInfo(ReceiveCarInfo &receive_car_info, zmq::socket_t &subscriber_receive_car) {
    zmq::message_t recv_message(sizeof(CarSendWatchtower));

    receive_car_info.comfirm_receipt = false;
    while (subscriber_receive_car.recv(&recv_message, ZMQ_DONTWAIT)) {
        memcpy(&receive_car_info.robo_car_info, recv_message.data(), sizeof(CarSendWatchtower));
        receive_car_info.comfirm_receipt = true;
    }

    if (receive_car_info.comfirm_receipt == true) {
        // 把单位从 m 转成 cm
        receive_car_info.robo_car_info.position.x *= 100;
        receive_car_info.robo_car_info.position.y *= 100;
    }

    return receive_car_info;
}


ReceiveOtherWatchtowerInfo Msgs::receiveOtherWatchtowerInfo(ReceiveOtherWatchtowerInfo &receive_tower_info, zmq::socket_t &subscriber_receive_other_tower) {
    // 单位 ms
    static auto last_time   = std::chrono::system_clock::now();    // 最后一次接收到哨岗信息的时间
    static auto now_time    = std::chrono::system_clock::now();    // 当前时间
    static auto diiff_time  = std::chrono::duration_cast<std::chrono::milliseconds>(last_time - now_time).count();
    
    zmq::message_t recv_message(sizeof(WatchtowerInfo));

    receive_tower_info.comfirm_receipt = false;
    while (subscriber_receive_other_tower.recv(&recv_message, ZMQ_DONTWAIT)) {
        memcpy(&receive_tower_info.other_tower_info, recv_message.data(), sizeof(WatchtowerInfo));
        receive_tower_info.comfirm_receipt = true;
        last_time = std::chrono::system_clock::now();   // 最后一次接收到哨岗信息的时间
    }

    now_time    = std::chrono::system_clock::now();     // 当前时间
    diiff_time  = std::chrono::duration_cast<std::chrono::milliseconds>(last_time - now_time).count();
    // 若最后一次副哨岗信息距今超过1000ms(1s), 判断副哨岗掉线 
    if (diiff_time < 1000) {
        receive_tower_info.other_tower_online = true;
    } else {
        receive_tower_info.other_tower_online = false;
    }

    return receive_tower_info;
}

ReceiveInfo Msgs::receiveInfo(ReceiveInfo &receive_info, zmq::socket_t &subscriber_receive_other_tower, zmq::socket_t &subscriber_receive_car_1, zmq::socket_t &subscriber_receive_car_2) {
    receive_info.receive_tower_info = receiveOtherWatchtowerInfo(receive_info.receive_tower_info, subscriber_receive_other_tower);
    receive_info.receive_car_1_info = receiveCarInfo(receive_info.receive_car_1_info, subscriber_receive_car_1);
    receive_info.receive_car_2_info = receiveCarInfo(receive_info.receive_car_2_info, subscriber_receive_car_2);

    return receive_info;
}

ReceiveInfo Msgs::get_receive_info() {
    return receiveInfo(this->receive_info_, this->subscriber_receive_other_tower, this->subscriber_receive_car_1, this->subscriber_receive_car_2);
}

void Msgs::send_info(WatchtowerInfo &tower_info, zmq::socket_t &publisher_send_info) {
    if (this->msgs_config_.watchtoer_identity == 0) {

        this->send_to_car_info_ = SendToCarInfo(
            tower_info.swap_color_mode,
            tower_info.discoloration_num,
            tower_info.blue1,
            tower_info.blue2,
            tower_info.red1,
            tower_info.red2,
            tower_info.gray_on_buff_F6,
            tower_info.gray_on_buff_F1
        );

        zmq::message_t send_message(sizeof(SendToCarInfo));
        memcpy(send_message.data(), &this->send_to_car_info_, sizeof(SendToCarInfo));
        publisher_send_info.send(send_message);

    } else if (this->msgs_config_.watchtoer_identity == 1) {

        zmq::message_t send_message(sizeof(WatchtowerInfo));
        memcpy(send_message.data(), &tower_info, sizeof(WatchtowerInfo));
        publisher_send_info.send(send_message);

    }
}


Msgs::Msgs(MsgsConfig msgs_config) {
    this->msgs_config_ = msgs_config;

    zmq::context_t send_context_cars_info(1);
    this->publisher_send_info = zmq::socket_t(send_context_cars_info, zmq::socket_type::pub);
    this->publisher_send_info.bind(this->msgs_config_.tower_self_ip);

    if (this->msgs_config_.watchtoer_identity == 0) {
        zmq::context_t receive_context_tower_info(1);
        this->subscriber_receive_other_tower = zmq::socket_t(receive_context_tower_info, ZMQ_SUB);
        this->subscriber_receive_other_tower.connect(this->msgs_config_.other_tower_ip);    // "tcp://192.168.1.154:5556"
        this->subscriber_receive_other_tower.setsockopt(ZMQ_SUBSCRIBE, "", 0);

        // 初始化 接收 car_1_info
        zmq::context_t receive_context_car_1_info(1);
        this->subscriber_receive_car_1 = zmq::socket_t(receive_context_car_1_info, ZMQ_SUB);
        this->subscriber_receive_car_1.connect(this->msgs_config_.car_1_ip);    // "tcp://192.168.1.66:5555"
        this->subscriber_receive_car_1.setsockopt(ZMQ_SUBSCRIBE, "", 0);

        // 初始化 接收 car_2_info
        zmq::context_t receive_context_car_2_info(1);
        this->subscriber_receive_car_2 = zmq::socket_t(receive_context_car_2_info, ZMQ_SUB);
        this->subscriber_receive_car_2.connect(this->msgs_config_.car_2_ip);    // "tcp://192.168.1.89:5555"
        this->subscriber_receive_car_2.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    }
}

Msgs::~Msgs() {
}