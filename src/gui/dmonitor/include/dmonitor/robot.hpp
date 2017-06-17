#pragma once
#include <QQuickPaintedItem>
#include "dtransmit/dtransmit.hpp"

namespace dmonitor {

class Robot : public QQuickPaintedItem {
    Q_OBJECT
    Q_PROPERTY(int id READ id WRITE setId NOTIFY idChanged)
    Q_PROPERTY(int port READ port WRITE setPort NOTIFY portChanged)
    Q_PROPERTY(QString address READ address WRITE setAddress NOTIFY addressChanged)


private:
    dtransmit::DTransmit* m_dtransmit;
    int m_id;
    int m_port;
    QString m_address;

public:
    Robot(QQuickItem* parent = 0);
    ~Robot();

    Q_INVOKABLE void init();

    void paint(QPainter *painter) override;
    int id() const;
    int port() const;

    QString address() const;

public slots:
    void setId(int id);
    void setPort(int port);

    void setAddress(QString address);

signals:
    void idChanged(int id);
    void portChanged(int port);
    void addressChanged(QString address);
};

} // namespace dmonitor
