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
        boost::asio::ip::udp::socket socket;
        uint8_t recvBuffer[1500];
        Foo(boost::asio::io_service& service, PORT port) : socket(service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port)) {
        }
    };

    std::string m_broadcastAddress;
    boost::asio::io_service m_service;

    std::thread m_t;
    boost::asio::ip::udp::endpoint m_remoteEndpoint;
    boost::asio::ip::udp::endpoint m_broadcastEndpoint;
    std::unordered_map<PORT, Foo> m_recvFoo;
    std::unordered_map<PORT, boost::asio::ip::udp::socket*> m_sendSockets;
};

template <typename ReadHandler>
void DTransmit::startRecv(PORT port, ReadHandler handler) {
    auto& buffer = m_recvFoo[port].recvBuffer;
    m_recvFoo[port].socket.async_receive_from(
        boost::asio::buffer(boost::asio::mutable_buffer((void*)&buffer,
                                           sizeof(buffer))), m_remoteEndpoint, handler
    );
}

template <typename ROSMSG>
void DTransmit::add_recv(PORT port, std::function<void(ROSMSG & )> callback) {
    using namespace boost::asio;

    m_recvFoo[port] = Foo(m_service, port);

    std::function<void(const boost::system::error_code& , std::size_t)> readHandler = [&](const boost::system::error_code& error, std::size_t bytesRecved) {
        ROSMSG msg;
        uint32_t serial_size = ros::serialization::serializationLength(msg);
        if(bytesRecved != serial_size) {
            ROS_WARN("DTransmit recved buffer with unexpected size: %d, expected: %d", bytesRecved, serial_size);
        } else {
            boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
            ros::serialization::IStream stream(buffer.get(), serial_size);
            // ros::serialization::deserialize(stream, msg);
            ros::serialization::Serializer<ROSMSG>::read(stream, msg);
            // client callback
            callback(msg);
        }
        startRecv(port, readHandler);
    };
    startRecv(port, readHandler);
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
        throw(ec);
    }

    m_sendSockets[port] = socket;
}

template <typename ROSMSG>
void DTransmit::send(PORT port, ROSMSG& rosmsg) {
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
