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
#include "dmotion/MotionInfo.h"
#include "dvision/utils.hpp"
#include "dmonitor/utils.hpp"

using namespace dvision;
using namespace cv;

namespace dmonitor {

Robot::Robot(QQuickItem *parent) : BaseObject(parent)
{
    m_lastRecvTime = QTime::currentTime().addSecs(-MAX_UNSEEN_SEC * 2);
}

void Robot::init()
{
    ros::Time::init();
    dvision::Frame::initEncoder();
    m_transmitter = new dtransmit::DTransmit(m_address.toStdString());

    // listen vision info
    m_transmitter->addRosRecv<dvision::VisionInfo>(dconstant::network::robotBroadcastAddressBase + m_robotId, std::bind(&Robot::onRecv, this, std::placeholders::_1));
    // listen delta data
    m_transmitter->addRosRecv<dmotion::MotionInfo>(dconstant::network::robotMotionBase + m_robotId, std::bind(&Robot::onRecvMotion, this, std::placeholders::_1));
    m_transmitter->startService();
}

Point3d Robot::realPos() {
    double scale = m_field->getScale();
    auto p = m_field->getOnRealCoordinate(QPointF(x() + m_triangleBBoxWidth / scale / 2, y() + m_triangleBBoxHeight / scale / 2));
    return Point3d(p.x(), p.y(), m_heading / 180 * M_PI);
}

void Robot::simModeUpdate()
{
    m_simBall->setVisible(true);
    m_ball->setVisible(false);
    if(isOnline()) {
        setWidth(m_triangleBBoxWidth);
        setHeight(m_triangleBBoxHeight);
        setVisible(true);


        double scale = m_field->getScale();
        double dx = m_motionInfo.deltaData.x;
        double dy = m_motionInfo.deltaData.y;
        double dt = m_motionInfo.deltaData.z;

        auto newReal = dvision::getOnGlobalCoordinate(realPos(), Point2d(dx, dy));
        auto realPos = m_field->getOnImageCoordiante(newReal);
        m_heading += dt;

        setX(realPos.x() - m_triangleBBoxWidth / scale / 2);
        setY(realPos.y() - m_triangleBBoxHeight / scale / 2);

        // update vision info according to xy
        m_simVisionInfo.robot_pos.x = newReal.x;
        m_simVisionInfo.robot_pos.y = newReal.y;

        // !? see ball

        // todo ........................................................................... this is fucking ..
        auto realBall = m_field->getOnRealCoordinate(m_simBall->x() + m_simBall->width() / 2, m_simBall->y() + m_simBall->height() / 2);
        m_simVisionInfo.ball_global.x = realBall.x();
        m_simVisionInfo.ball_global.y = realBall.y();

        auto ballField = dvision::getOnRobotCoordinate(this->realPos(), Point2d(realBall.x(), realBall.y()));
        m_simVisionInfo.ball_field.x = ballField.x;
        m_simVisionInfo.ball_field.y = ballField.y;

        m_simVisionInfo.see_ball = true;
        m_transmitter->sendRos(dconstant::network::monitorBroadcastAddressBase + m_robotId, m_simVisionInfo);
    } else {
        setVisible(false);
    }
}

void Robot::monitorModeUpdate()
{
    m_simBall->setVisible(false);
    if(isOnline()) {
        setX(0);
        setY(0);
        setWidth(m_field->width());
        setHeight(m_field->height());
        setVisible(true);
        // !? set ball
        m_ball->setVisible(true);
    } else {
        setVisible(false);
        m_ball->setVisible(false);
    }
}

void Robot::drawMyself(QPainter* painter) {
    double scale = m_field->getScale();
    if(m_isMonitor) {
        auto imgPos = m_field->getOnImageCoordiante(m_realPos);
        painter->translate(imgPos.x(), imgPos.y());
    } else {
        painter->translate(m_triangleBBoxWidth / scale / 2 , m_triangleBBoxHeight / scale / 2);
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

void Robot::onRecv(dvision::VisionInfo &msg)
{
    m_monVisionInfo = msg;
    m_heading = msg.robot_pos.z * 180.0 / M_PI;
    m_realPos = QPointF(msg.robot_pos.x, msg.robot_pos.y);
    m_lastRecvTime = QTime::currentTime();
    m_viewRange->setVisionInfo(msg);
    if(m_ball)
        m_ball->setPos(QPointF(msg.ball_global.x, msg.ball_global.y));
}


void Robot::onRecvMotion(dmotion::MotionInfo& msg) {
    m_motionInfo = msg;
    m_lastRecvTime = QTime::currentTime();
}



















































































































///////////////////////////////////////////////////////////// setter & getter
void Robot::reset() {
    setX(m_field->width() / 2 - 1 / m_field->getScale() * m_triangleBBoxWidth / 2);
    setY(m_field->height() / 2 - 1 / m_field->getScale() * m_triangleBBoxHeight / 2);
    m_heading = 0;
}
bool Robot::isOnline() {
    QTime now = QTime::currentTime();
    int last = m_lastRecvTime.secsTo(now);
    bool online;
    if(last > MAX_UNSEEN_SEC) {
        online = false;
    } else {
        online = true;
    }
    if(online != m_online) {
        m_online = online;
        emit onlineChanged(online);
    }
    return online;
}


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

void Robot::setSimBall(Ball *simBall)
{
    m_simBall = simBall;
}

void Robot::setViewRange(ViewRange *viewRange)
{
    if (m_viewRange == viewRange)
        return;

    m_viewRange = viewRange;
    emit viewRangeChanged(viewRange);
}
bool Robot::online() const {
    return m_online;
}


Robot::~Robot() {
    delete m_transmitter;
}
Ball *Robot::simBall() const
{
    return m_simBall;
}

ViewRange *Robot::viewRange() const
{
    return m_viewRange;
}

} // namespace dmonitor

