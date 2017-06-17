// Created on: June 16, 2017
//     Author: Wenxing Mei <mwx37mwx@gmail.com>

#pragma once
#include <memory>
#include <vector>
#include <string>
#include <boost/asio/buffer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/udp.hpp>
#include <unordered_map>
#include <thread>
#include <functional>
#include <ros/ros.h>

// http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes

namespace dtransmit {
class DTransmit {
    typedef int PORT;
public:
    explicit DTransmit(std::string address);
    ~DTransmit();

    template<typename ROSMSG>
    void add_recv(PORT p, std::function<void(ROSMSG &)> f);

    template <typename ROSMSG>
    void send(PORT p, ROSMSG&);

private:
    template<typename ReadHandler>
    void startRecv(PORT p, ReadHandler handler);
    void createSendSocket(PORT);
    void sendBuffer(boost::asio::ip::udp::socket*, const void* buffer, std::size_t size);

    // maybe use pair
    struct Foo {
        boost::asio::ip::udp::socket* socket;
        std::function<void(const boost::system::error_code& , std::size_t)> readHandler;
        uint8_t recvBuffer[1500];
        Foo() {
        }

        Foo(boost::asio::io_service& service, PORT port) {
            socket = new boost::asio::ip::udp::socket(service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port));
        }
    };

    std::string m_broadcastAddress;
    boost::asio::io_service m_service;

    std::thread* m_t;
    boost::asio::ip::udp::endpoint m_remoteEndpoint;
    boost::asio::ip::udp::endpoint m_broadcastEndpoint;
    std::unordered_map<PORT, Foo> m_recvFoo;
    std::unordered_map<PORT, boost::asio::ip::udp::socket*> m_sendSockets;
};

template <typename ReadHandler>
void DTransmit::startRecv(PORT port, ReadHandler handler) {
    m_recvFoo[port].socket->async_receive_from(
        boost::asio::buffer(boost::asio::mutable_buffer((void*)&m_recvFoo[port].recvBuffer,
                                           sizeof(m_recvFoo[port].recvBuffer))), m_remoteEndpoint, handler
    );
}

template <typename ROSMSG>
void DTransmit::add_recv(PORT port, std::function<void(ROSMSG & )> callback) {
    // std::cout << "Start add_recv" << std::endl;
    using namespace boost::asio;
    m_recvFoo[port] = Foo(m_service, port);

    m_recvFoo[port].readHandler = [=](const boost::system::error_code& error, std::size_t bytesRecved) {
        if(error) {
            ROS_INFO("Error in recv: %s", error.message().c_str());
        } else {
            ROSMSG msg;

            ros::serialization::IStream stream((uint8_t*)m_recvFoo[port].recvBuffer, bytesRecved);
            ros::serialization::Serializer<ROSMSG>::read(stream, msg);
            // client callback
            callback(msg);
        }

        startRecv(port, m_recvFoo[port].readHandler);
    };
    startRecv(port, m_recvFoo[port].readHandler);

// TODO(MWX): better solution

    if(m_t) {
        m_service.stop();
        // m_t->join();
        // delete m_t;
    }

    
    m_t = new std::thread([&]() {
        m_service.reset();
        m_service.run();
    });
    m_t->detach();

    // std::cout << "Add recv end" << std::endl;
    
}

void DTransmit::createSendSocket(PORT port) {
    using namespace boost::asio;
    ip::udp::endpoint broadcastEndpoint(ip::address::from_string(m_broadcastAddress), port);
    auto socket = new ip::udp::socket(m_service, ip::udp::v4());
    socket->set_option(socket_base::broadcast(true));

    boost::system::error_code ec;
    socket->connect(broadcastEndpoint, ec);
    if(ec) {
        ROS_ERROR("DTransmit create send socket error: %s", ec.message().c_str());
    }

    m_sendSockets[port] = socket;
}

template <typename ROSMSG>
void DTransmit::send(PORT port, ROSMSG& rosmsg) {
    ROS_INFO("Sending through %d", port);
    if(!m_sendSockets.count(port)) {
       createSendSocket(port);
    }
    // serialize rosmsg
    uint32_t serial_size = ros::serialization::serializationLength(rosmsg);
    std::unique_ptr<uint8_t> buffer(new uint8_t[serial_size]);

    ros::serialization::OStream stream(buffer.get(), serial_size);
    ros::serialization::serialize(stream, rosmsg);

    sendBuffer(m_sendSockets[port], buffer.get(), serial_size);
}

void DTransmit::sendBuffer(boost::asio::ip::udp::socket* socket, const void* buffer, std::size_t size) {
    boost::system::error_code ec;
    socket->send(boost::asio::buffer(buffer, size), 0, ec);
    if(ec) {
        ROS_ERROR("DTransmit can't send buffer: %s", ec.message().c_str());
    }
}

} // namespace dtransmit
