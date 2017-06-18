#pragma once
#include <QQuickPaintedItem>
#include "dtransmit/dtransmit.hpp"
#include "dmonitor/field.hpp"
#include "dvision/VisionInfo.h"

namespace dmonitor {

class Robot : public QQuickPaintedItem {
    Q_OBJECT
    Q_PROPERTY(int robotId READ robotId WRITE setRobotId)
    Q_PROPERTY(QString address READ address WRITE setAddress)
    Q_PROPERTY(Field* field READ field WRITE setField)
    Q_PROPERTY(bool isMonitor READ isMonitor WRITE setIsMonitor)


public:
    Robot(QQuickItem* parent = 0);
    ~Robot();

    Q_INVOKABLE void init();
    void paint(QPainter *painter) override;

    // qml read
    int robotId() const;
    QString address() const;
    Field* field() const;

    void drawRobot(QPainter *painter);
    void simUpdate();
    void moUpdate();

    bool isMonitor() const;

public slots:
    void setRobotId(int robotId);
    void setAddress(QString address);
    void setField(Field* field);
    void onRecv(dvision::VisionInfo& msg);
    void onxyChanged();
    void setIsMonitor(bool isMonitor);

private:
    bool m_isMonitor = true;
    dvision::VisionInfo m_simVisionInfo;
    Field* m_field;
    dtransmit::DTransmit* m_transmitter;
    int m_robotId;
    QString m_address;

    QPointF m_robotPos;
    double m_heading;
    QPointF m_ballPos;
    QColor m_color = QColor(255, 167, 0);
};

} // namespace dmonitor
