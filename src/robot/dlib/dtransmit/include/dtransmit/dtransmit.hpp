// Created on: June 16, 2017
//     Author: Wenxing Mei <mwx37mwx@gmail.com>

#pragma once
#include <vector>
#include <string>
#include <boost/asio/buffer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/udp.hpp>
#include <unordered_map>
#include <functional>

// http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes

namespace dtransmit {
class DTransmit {
    typedef int PORT;
public:
    DTransmit();
    ~DTransmit();

    template<typename MSG>
    void add_recv(PORT p, std::function<void(MSG &)> f);

    template <typename MSG>
    void send(PORT p, MSG&);

private:
   std::unordered_map<PORT, boost::asio::ip::udp::socket> sockets;
};
}
