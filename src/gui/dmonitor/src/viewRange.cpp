#include "dmonitor/viewRange.hpp"
#include <QPainter>

namespace dmonitor {

ViewRange::ViewRange(QQuickItem *parent) : BaseObject(parent)
{
}

void ViewRange::drawMyself(QPainter *painter)
{
    if(m_visionInfo.viewRange.size() != 4){
        return;
    }

    setX(0);
    setY(0);
    setWidth(m_field->width());
    setHeight(m_field->height());

    QPointF a(m_visionInfo.viewRange[0].x, m_visionInfo.viewRange[0].y);
    QPointF b(m_visionInfo.viewRange[1].x, m_visionInfo.viewRange[1].y);
    QPointF c(m_visionInfo.viewRange[2].x, m_visionInfo.viewRange[2].y);
    QPointF d(m_visionInfo.viewRange[3].x, m_visionInfo.viewRange[3].y);

    qDebug() << a << b << c << d;

    auto aa = m_field->getOnImageCoordiante(a);
    auto bb = m_field->getOnImageCoordiante(b);
    auto cc = m_field->getOnImageCoordiante(c);
    auto dd = m_field->getOnImageCoordiante(d);

    std::vector<QPointF> points {
        aa, bb, cc, dd
    };

    QPen pen(QColor(Qt::yellow), 0);
    painter->setPen(pen);
    painter->setBrush(QBrush());
    painter->drawPolygon(points.data(), points.size());
}

void ViewRange::setVisionInfo(dvision::VisionInfo info)
{
    m_visionInfo = info;
}

}
