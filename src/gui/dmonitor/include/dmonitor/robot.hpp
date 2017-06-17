#pragma once
#include <QQuickPaintedItem>
#include "dtransmit/dtransmit.hpp"
#include "dvision/VisionInfo.h"

namespace dmonitor {

class Robot : public QQuickPaintedItem {
    Q_OBJECT
    Q_PROPERTY(int robotId READ robotId WRITE setRobotId)
    Q_PROPERTY(QString address READ address WRITE setAddress)


public:
    Robot(QQuickItem* parent = 0);
    ~Robot();

    Q_INVOKABLE void init();
    void paint(QPainter *painter) override;

    // qml read
    int robotId() const;
    QString address() const;

public slots:
    void setRobotId(int robotId);
    void setAddress(QString address);


private:
    dtransmit::DTransmit* m_transmitter;
    int m_robotId;
    QString m_address;
    dvision::VisionInfo m_visionInfo;
};

} // namespace dmonitor
