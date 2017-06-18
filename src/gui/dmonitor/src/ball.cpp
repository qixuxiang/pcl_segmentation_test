#include "dmonitor/ball.hpp"
#include <QPainter>

namespace dmonitor {
Ball::Ball(QQuickItem *parent) : BaseObject(parent) {
}

void Ball::init()
{
   setWidth(m_diameter);
   setHeight(m_diameter);
}

void Ball::setPos(QPointF pos)
{
   m_realPos = pos;
}

void Ball::drawMyself(QPainter* painter)
{
    QRectF rec(0, 0, width(), height());
    painter->setBrush(m_color);
    painter->setPen(QPen(m_color, 0));
    painter->drawEllipse(rec);
}



} // namespace dmonitor
