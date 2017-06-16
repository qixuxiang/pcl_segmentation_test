#include "dtransmit/dtransmit.hpp"
#include <algorithm>

using namespace std;
using namespace boost::asio;

namespace dtransmit {
DTransmit::DTransmit(string address)
    : m_broadcastAddress(address), m_service()
{
    m_t = thread([&]() {
       m_service.run();
    });
}

DTransmit::~DTransmit() {
    m_service.stop();
    m_t.join();
}
}
