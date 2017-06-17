#include "dmonitor/robot.hpp"
#include <QDebug>

namespace dmonitor {

Robot::Robot(QQuickItem *parent)
{
}

Robot::~Robot()
{

}

void Robot::init()
{
    m_dtransmit = new dtransmit::DTransmit(m_address.toStdString());
    // add ROSRECV
    // dtransmit.startService();
}


void Robot::paint(QPainter *painter)
{
}


// setter & getter

int Robot::id() const
{
    return m_id;
}

int Robot::port() const
{
    return m_port;
}

QString Robot::address() const
{
    return m_address;
}

void Robot::setId(int id)
{
    if (m_id == id)
        return;

    m_id = id;
    emit idChanged(id);
}

void Robot::setPort(int port)
{
    if (m_port == port)
        return;

    m_port = port;
    emit portChanged(port);
}

void Robot::setAddress(QString address)
{
    if (m_address == address)
        return;

    m_address = address;
    emit addressChanged(address);
}

} // namespace dmonitor
