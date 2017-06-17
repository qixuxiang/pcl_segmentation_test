#include "dmonitor/robot.hpp"
#include <QPainter>
#include <QDebug>

namespace dmonitor {

Robot::Robot(QQuickItem *parent) : QQuickPaintedItem(parent)
{
}

Robot::~Robot()
{

}

void Robot::init()
{
    m_transmitter = new dtransmit::DTransmit(m_address.toStdString());

    int port = 2000 + m_robotId;
    qDebug() << "Listening at " << port;
    m_transmitter->addRosRecv<dvision::VisionInfo>(port, [&](dvision::VisionInfo& msg) {
        m_visionInfo = msg;
    });
    m_transmitter->startService();
}


void Robot::paint(QPainter *painter)
{
    if(m_visionInfo.robot_pos.x != 0) {
        this->setX(m_visionInfo.robot_pos.x + 450 - width() / 2);
        this->setY(m_visionInfo.robot_pos.y + 300 - height() / 2);
        qDebug() << m_visionInfo.robot_pos.x << " " << m_visionInfo.robot_pos.y;
    }

    // draw according to visioninfo
    QRectF rectangle(0, 0, width(), height());
    painter->fillRect(rectangle, Qt::black);
    painter->setBrush(QBrush(QColor(191, 255, 0)));
    painter->setRenderHint(QPainter::Antialiasing);
    painter->drawEllipse(rectangle);
}

///////////////// setter & getter

int Robot::robotId() const
{
    return m_robotId;
}

QString Robot::address() const
{
    return m_address;
}

void Robot::setRobotId(int id)
{
    qDebug() << "set robot id" << id << endl;
    if (m_robotId == id)
        return;

    m_robotId = id;
}

void Robot::setAddress(QString address)
{
    if (m_address == address)
        return;
    m_address = address;
}

} // namespace dmonitor
