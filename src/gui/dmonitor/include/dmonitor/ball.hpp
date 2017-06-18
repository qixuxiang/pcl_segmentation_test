#pragma once
#include "dmonitor/baseObject.hpp"
#include "dmonitor/field.hpp"
#include "dconfig/dconstant.hpp"
#include "dmonitor/baseObject.hpp"
namespace dmonitor {

class Ball : public BaseObject {
    Q_OBJECT

public:
    Ball(QQuickItem* parent = 0);
    void init() override;
    void drawMyself(QPainter*) override;
    void setPos(QPointF pos);

private:
    int m_id;
    QPointF m_realPos;
    Field* m_field;

    int m_diameter = dconstant::ballSize::diameter;
    QColor m_color = QColor(255, 64, 64);
};

} // namespace dmonitor
