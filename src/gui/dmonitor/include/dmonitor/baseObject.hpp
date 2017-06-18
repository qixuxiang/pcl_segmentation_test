#pragma once
#include <QQuickPaintedItem>
#include "field.hpp"

namespace dmonitor {
class BaseObject : public QQuickPaintedItem {
    Q_OBJECT
    Q_PROPERTY(Field* field READ field WRITE setField)
    Q_PROPERTY(bool isMonitor READ isMonitor WRITE setIsMonitor)
    Q_PROPERTY(bool enabled READ enabled WRITE setEnable)

public:
    BaseObject(QQuickItem* parent = 0);
    virtual ~BaseObject();
    Q_INVOKABLE virtual void init();

    // qml read
    Field* field() const;
    bool isMonitor() const;
    void paint(QPainter *painter) override;

    bool enabled() const;

public slots:
    void setField(Field* field);
    void setIsMonitor(bool isMonitor);
    void setEnable(bool enabled);


    // Function maybe re-implemented
    virtual void onxyChanged();
    virtual void monitorModeUpdate();
    virtual void simModeUpdate();
    virtual void drawMyself(QPainter* painter) = 0;

protected:
    Field* m_field = nullptr;
    bool m_isMonitor;
    QPointF m_realPos;
    bool m_enabled = true;

};
} // namespace dmonitor
