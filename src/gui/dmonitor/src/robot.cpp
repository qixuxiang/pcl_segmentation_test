// TODO(MWX): draw ball
// TODO(MWX): set mouseare to robot, and set scene width and height same as field, so we can draw balls on the same painter
// draw robot by drawing in different position but not setX() and setY()

#include "dmonitor/robot.hpp"
#include <QPainter>
#include <QDebug>
#include <functional>
#include <vector>
#include "dconfig/dconstant.hpp"

namespace dmonitor {

Robot::Robot(QQuickItem *parent) : QQuickPaintedItem(parent)
{
    connect(this, &Robot::xChanged,
            this, &Robot::onxyChanged);
    connect(this, &Robot::yChanged,
            this, &Robot::onxyChanged);
}

Robot::~Robot()
{
    delete m_transmitter;
}

void Robot::init()
{
    m_transmitter = new dtransmit::DTransmit(m_address.toStdString());
    //m_transmitter = new dtransmit::DTransmit("127.0.0.1");

    int port = dconstant::network::robotBroadcastAddressBase + m_robotId;
    qDebug() << "Listening at" << port;
    m_transmitter->addRosRecv<dvision::VisionInfo>(port, std::bind(&Robot::onRecv, this, std::placeholders::_1));
    m_transmitter->startService();

    setWidth(40);
    setHeight(40);

//    for(int i = 0; ; ++i) {
//        m_transmitter->sendRos(dconstant::network::monitorBroadcastAddressBase + m_robotId, m_simVisionInfo);
//    }
}

void Robot::paint(QPainter *painter)
{
    if(!m_field){
        qDebug() << "Robot field not set!" << endl;
        return;
    }

    painter->setRenderHint(QPainter::Antialiasing);

    if(!m_isMonitor) {
        simUpdate();
    } else {
        moUpdate();
    }
    drawRobot(painter);
}

void Robot::simUpdate()
{
    // update vision info according to xy
    auto realRobot = m_field->getOnRealCoordinate(QPointF(x() + width() / 2, y() + width() / 2));
    m_simVisionInfo.robot_pos.x = realRobot.x();
    m_simVisionInfo.robot_pos.y = realRobot.y();

    if(!m_isMonitor) {
        m_transmitter->sendRos(dconstant::network::monitorBroadcastAddressBase + m_robotId, m_simVisionInfo);
    }
}

void Robot::moUpdate()
{
    // draw robot according to localized position
    auto imgPos = m_field->getOnImageCoordiante(m_robotPos);
    setX(imgPos.x() - width() / 2);
    setY(imgPos.y() - height() / 2);
}


void Robot::drawRobot(QPainter* painter) {
    painter->translate(width() / 2, height() / 2);
    painter->rotate(90 - m_heading);
    std::vector<QPointF> points {
        QPointF(0, -15),
        QPointF(10, 10),
        QPointF(-10, 10)
    };
    QPen pen(m_color, 0);

    painter->setClipping(false);
    painter->setPen(pen);
    painter->setPen(m_color);
    painter->setBrush(m_color);
    painter->drawPolygon(points.data(), points.size());

    // back
    painter->rotate(m_heading - 90);
    painter->setPen(Qt::black);
    painter->drawText(QPointF(-5, 5), QString("%1").arg(m_robotId));
    painter->translate(-width() / 2, -height() / 2);
}



void Robot::onRecv(dvision::VisionInfo &msg)
{
    // TODO(MWX): delta data
    if(m_isMonitor) {
        qDebug() << "recv dvision info";
        m_heading = msg.robot_pos.z;
        m_robotPos = QPointF(msg.robot_pos.x, msg.robot_pos.y);
        m_ballPos = QPointF(msg.ball_global.x, msg.ball_global.y);
    }
}


void Robot::onxyChanged()
{
}















///////////////////////////////////////////////////////////// setter & getter

int Robot::robotId() const
{
    return m_robotId;
}

QString Robot::address() const
{
    return m_address;
}

Field *Robot::field() const
{
    return m_field;
}

void Robot::setRobotId(int id)
{
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

void Robot::setField(Field *field)
{
    m_field = field;
}


void Robot::setIsMonitor(bool isMonitor)
{
    if (m_isMonitor == isMonitor)
        return;

    m_isMonitor = isMonitor;
}
bool Robot::isMonitor() const
{
    return m_isMonitor;
}

} // namespace dmonitor

