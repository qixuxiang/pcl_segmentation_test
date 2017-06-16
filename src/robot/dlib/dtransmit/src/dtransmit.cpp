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
//    m_service.stop();
//    m_t.join();
}
}
