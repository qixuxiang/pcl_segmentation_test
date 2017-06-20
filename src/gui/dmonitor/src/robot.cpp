// TODO(MWX): draw ball
// TODO(MWX): set mouseare to robot, and set scene width and height same as field, so we can draw balls on the same painter
// draw robot by drawing in different position but not setX() and setY()

#include "dmonitor/robot.hpp"
#include <QPainter>
#include <QDebug>
#include <functional>
#include <vector>
#include "dconfig/dconstant.hpp"
#include "dvision/frame.hpp"

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
    ros::Time::init();
    dvision::Frame::initEncoder();
    m_transmitter = new dtransmit::DTransmit(m_address.toStdString());
    //m_transmitter = new dtransmit::DTransmit("127.0.0.1");

    int port = dconstant::network::robotBroadcastAddressBase + m_robotId;
    qDebug() << "Listening at" << port;
    m_transmitter->addRosRecv<dvision::VisionInfo>(port, std::bind(&Robot::onRecv, this, std::placeholders::_1));
    m_transmitter->startService();
}

void Robot::simModeUpdate()
{
    double scale = m_field->getScale();
    setWidth(m_triangleBBoxWidth / scale);
    setHeight(m_triangleBBoxHeight / scale);

    setVisible(true);
    m_ball->setVisible(true);

    // update vision info according to xy
    auto realRobot = m_field->getOnRealCoordinate(QPointF(x() + m_triangleBBoxWidth / 2, y() + m_triangleBBoxHeight / 2));
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


// monitor mode: set(0, 0), draw(x, y)
// sim mode: set(x, y), draw(0, 0)

void Robot::monitorModeUpdate()
{
    setX(0);
    setY(0);
    setWidth(m_field->width());
    setHeight(m_field->height());

    QTime now = QTime::currentTime();
    int last = m_lastRecvTime.secsTo(now);
    if(last > MAX_UNSEEN_SEC) {
        m_online = false;
        setVisible(false);
        m_ball->setVisible(false);
        emit onlineChanged(m_online);
    } else {
        setVisible(true);
        m_online = true;
        emit onlineChanged(m_online);

        if(m_simVisionInfo.see_ball)
            m_ball->setVisible(true);
        else
            m_ball->setVisible(false);
    }
}



void Robot::drawMyself(QPainter* painter) {
    double scale = m_field->getScale();
    if(m_isMonitor) {
        auto imgPos = m_field->getOnImageCoordiante(m_realPos);
        painter->translate(imgPos.x(), imgPos.y());
    } else {
        painter->translate(-m_triangleBBoxWidth / scale / 2, -m_triangleBBoxHeight / scale / 2);
    }

    painter->scale(1 / scale, 1 / scale);
    painter->rotate(90 - m_heading);

    // draw robot
    std::vector<QPointF> points {
        QPointF(0, -15),
        QPointF(10, 10),
        QPointF(-10, 10)
    };

    QPen pen(m_color, 0);
    painter->setPen(pen);
    painter->setBrush(m_color);
    painter->drawPolygon(points.data(), points.size());

    // back
    painter->rotate(m_heading - 90);
    painter->setPen(Qt::black);
    painter->drawText(QPointF(-5, 5), QString("%1").arg(m_robotId));
    painter->scale(scale, scale);
    if(m_isMonitor) {
        auto imgPos = m_field->getOnImageCoordiante(m_realPos);
        painter->translate(-imgPos.x(), -imgPos.y());
        drawLines(painter);
        drawCircle(painter);
        drawView(painter);
    } else {
        painter->translate(m_triangleBBoxWidth / scale / 2, m_triangleBBoxHeight / scale / 2);
    }
}

void Robot::drawLines(QPainter* painter) {
    foreach (const dvision::Line& line, m_monVisionInfo.lines) {
        auto& p1 = line.endpoint1;
        auto& p2 = line.endpoint2;

        auto pp1 = m_field->getOnImageCoordiante(QPointF(p1.x, p1.y));
        auto pp2 = m_field->getOnImageCoordiante(QPointF(p2.x, p2.y));

        painter->setPen(QPen(Qt::white, 2));
        painter->drawLine(pp1, pp2);
    }
}

void Robot::drawCircle(QPainter* painter) {
    auto circleCenter = m_field->getOnImageCoordiante(QPointF(m_monVisionInfo.circle_field.x, m_monVisionInfo.circle_field.y));

    double circleDiamater = dconstant::geometry::centerCircleDiameter / m_field->getScale();
    QRectF foo(circleCenter.x() - circleDiamater / 2,circleCenter.y() - circleDiamater / 2, circleDiamater, circleDiamater);
    painter->setPen(QPen(Qt::yellow, 2));
    painter->setBrush(QBrush());
    painter->drawEllipse(foo);
}

void Robot::drawView(QPainter* painter) {
    if(m_monVisionInfo.viewRange.size() != 4)
        return;

    QPointF a(m_monVisionInfo.viewRange[0].x, m_monVisionInfo.viewRange[0].y);
    QPointF b(m_monVisionInfo.viewRange[1].x, m_monVisionInfo.viewRange[1].y);
    QPointF c(m_monVisionInfo.viewRange[2].x, m_monVisionInfo.viewRange[2].y);
    QPointF d(m_monVisionInfo.viewRange[3].x, m_monVisionInfo.viewRange[3].y);

    auto aa = m_field->getOnImageCoordiante(a);
    auto bb = m_field->getOnImageCoordiante(b);
    auto cc = m_field->getOnImageCoordiante(c);
    auto dd = m_field->getOnImageCoordiante(d);

    std::vector<QPointF> points {
        aa, bb, cc, dd
    };

    QPen pen(m_color, 0);
    painter->setPen(pen);
    painter->setBrush(QBrush());
    painter->drawPolygon(points.data(), points.size());

}

bool Robot::online() const
{
    return m_online;
}


void Robot::onRecv(dvision::VisionInfo &msg)
{
    // TODO(MWX): delta data
    if(m_isMonitor) {
        m_monVisionInfo = msg;
        m_heading = msg.robot_pos.z * 180.0 / M_PI;
        auto robotPos = QPointF(msg.robot_pos.x, msg.robot_pos.y);

//        if(robotPos.x() != 0.0 && robotPos.y() != 0.0) {
        if(true) {
            m_realPos = robotPos;
            m_lastRecvTime = QTime::currentTime();
        }

        if(m_ball)
            m_ball->setPos(QPointF(msg.ball_global.x, msg.ball_global.y));
    }
}



///////////////////////////////////////////////////////////// setter & getter


QString Robot::address() const
{
    return m_address;
}

Ball* Robot::ball() const
{
    return m_ball;
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

void Robot::setOnline(bool online)
{
    if (m_online == online)
        return;

    m_online = online;
    emit onlineChanged(online);
}

} // namespace dmonitor

