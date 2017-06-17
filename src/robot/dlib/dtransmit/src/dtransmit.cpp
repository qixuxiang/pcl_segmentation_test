#include "dtransmit/dtransmit.hpp"
#include <algorithm>

using namespace std;
using namespace boost::asio;

namespace dtransmit {
DTransmit::DTransmit(string address)
    : m_broadcastAddress(address), m_service()
{
}

DTransmit::~DTransmit() {
    m_service.stop();
    for(auto& p : m_sendSockets) {
        ip::udp::socket* s = p.second;
        if(s) {
            if(s->is_open())
                s->close();
            delete s;
        }
    }

    for(auto& p : m_recvFoo) {
        ip::udp::socket* s = p.second.socket;
        if(s) {
            if(s->is_open())
                s->close();

            delete s;
        }
    }

    m_service.stop();
    if(m_t.joinable())
        m_t.join();
//    for_each(m_sendSockets.begin(), m_sendSockets.end(), [](pair<PORT, ip::udp::socket*>& p) {
//        ip::udp::socket* s = p.second;
//        if(s->is_open())  {
//            s->close();
//        }
//        delete s;
//    });
    // TODO(MWX): delete all socket
    //if(m_t) {
        // m_t->join();
        //delete m_t; 
    //}
}
}
