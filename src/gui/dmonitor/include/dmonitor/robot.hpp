#pragma once
#include "dmonitor/baseObject.hpp"
#include "dmonitor/ball.hpp"
#include "dtransmit/dtransmit.hpp"
#include "dvision/VisionInfo.h"
#include <QTime>

namespace dmonitor {

class Robot : public BaseObject {
    Q_OBJECT
    Q_PROPERTY(int robotId READ robotId WRITE setRobotId)
    Q_PROPERTY(QString address READ address WRITE setAddress)
    Q_PROPERTY(Ball* ball READ ball WRITE setBall)

public:
    Robot(QQuickItem* parent = 0);
    ~Robot();

    // OVERRIDE FUNCTIONS
    void simModeUpdate() override;
    void monitorModeUpdate() override;
    void drawMyself(QPainter* painter) override;
    void drawLines(QPainter *painter);
    Q_INVOKABLE void init() override;

    // qml read
    int robotId() const;
    QString address() const;
    Ball* ball() const;

    void drawCircle(QPainter *painter);
public slots:
    void setRobotId(int robotId);
    void setAddress(QString address);
    void onRecv(dvision::VisionInfo& msg);
    void setBall(Ball* ball);

private:
    dtransmit::DTransmit* m_transmitter;
    dvision::VisionInfo m_simVisionInfo;
    dvision::VisionInfo m_monVisionInfo;

    QTime m_lastRecvTime;
    Ball* m_ball = nullptr;
    int m_robotId;
    QString m_address;

    QPointF m_realPos;
    QPointF m_ballPos;
    double m_heading;
    int m_triangleBBoxWidth = 40;
    int m_triangleBBoxHeight = 40;
    QColor m_color = QColor(255, 167, 0);
    const int MAX_UNSEEN_SEC = 15;
};

} // namespace dmonitor
