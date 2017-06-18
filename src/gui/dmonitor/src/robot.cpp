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

Robot::Robot(QQuickItem *parent) : BaseObject(parent)
{
    m_lastRecvTime = QTime::currentTime().addSecs(-MAX_UNSEEN_SEC * 2);
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
}

void Robot::simModeUpdate()
{
    setVisible(true);
    m_ball->setVisible(true);

    // update vision info according to xy
    auto realRobot = m_field->getOnRealCoordinate(QPointF(x() + width() / 2, y() + height() / 2));
    m_simVisionInfo.robot_pos.x = realRobot.x();
    m_simVisionInfo.robot_pos.y = realRobot.y();
    if(!m_ball) {
        qDebug() << "ball not set" << endl;
    } else {
        auto realBall = m_field->getOnRealCoordinate(QPointF(m_ball->x() + m_ball->width() / 2, m_ball->y() + m_ball->height() /2));
        m_simVisionInfo.ball_global.x = realBall.x();
        m_simVisionInfo.ball_global.y = realBall.y();
    }
    m_transmitter->sendRos(dconstant::network::monitorBroadcastAddressBase + m_robotId, m_simVisionInfo);
}

void Robot::monitorModeUpdate()
{
    auto imgPos = m_field->getOnImageCoordiante(m_realPos);
    setX(imgPos.x() - width() / 2);
    setY(imgPos.y() - height() / 2);

    QTime now = QTime::currentTime();
    int last = m_lastRecvTime.secsTo(now);
    if(last > MAX_UNSEEN_SEC) {
        setVisible(false);
        m_ball->setVisible(false);
    } else {
        setVisible(true);

        if(m_simVisionInfo.see_ball)
            m_ball->setVisible(true);
        else
            m_ball->setVisible(false);
    }
}


void Robot::drawMyself(QPainter* painter) {
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
        m_lastRecvTime = QTime::currentTime();
        m_heading = msg.robot_pos.z;
        m_realPos = QPointF(msg.robot_pos.x, msg.robot_pos.y);
        if(m_ball)
            m_ball->setPos(QPointF(msg.ball_global.x, msg.ball_global.y));
    }
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

Ball* Robot::ball() const
{
    return m_ball;
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

void Robot::setBall(Ball* ball)
{
    if (m_ball == ball)
        return;

    m_ball = ball;
}

} // namespace dmonitor

