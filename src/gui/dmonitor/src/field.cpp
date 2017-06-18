#include "dmonitor/field.hpp"
#include <QDebug>
#include <QPainter>
namespace dmonitor {

Field::Field(QQuickItem *parent) : QQuickPaintedItem(parent)
{
}

void Field::paint(QPainter *painter)
{
   setWidth(m_imageWidth);
   setHeight(m_imageHeight);
   drawField(painter);
}

void Field::drawField(QPainter *painter)
{
    // antialiasing
    painter->setRenderHint(QPainter::Antialiasing);

    // draw background
    painter->fillRect(0, 0, m_imageWidth, m_imageHeight, m_grassGreen);


    // draw border
    QPointF upleftCorner = getOnImageCoordiante(QPoint(-m_fieldLength / 2, m_fieldWidth / 2));
    QRectF border(upleftCorner, flipFromOrigin(upleftCorner));

    QPen pen(Qt::white, m_lineWidth);
    painter->setPen(pen);
    painter->drawRect(border);

    // draw goal
    QPointF leftGoalUpleft = getOnImageCoordiante(QPointF(-m_fieldLength / 2 - m_goalDepth, m_goalWidth /2));
    QPointF leftGoalDownRight = getOnImageCoordiante(QPointF(-m_fieldLength / 2, -m_goalWidth / 2));
    QRectF leftGoal(leftGoalUpleft, leftGoalDownRight);
    painter->drawRect(leftGoal);
    painter->drawRect(flipFromOrigin(leftGoal));

    // draw goalArea
    QPointF leftGoalAreaUpLeft = getOnImageCoordiante(QPointF(-m_fieldLength / 2, m_goalAreaWidth / 2));
    QPointF leftGoalAreaDownRight = getOnImageCoordiante(QPointF(-m_fieldLength / 2 + m_goalAreaLength, -m_goalAreaWidth / 2));
    QRectF leftGoalArea(leftGoalAreaUpLeft, leftGoalAreaDownRight);
    painter->drawRect(leftGoalArea);
    painter->drawRect(flipFromOrigin(leftGoalArea));


    // draw center line
    QPointF centerLineUp = getOnImageCoordiante(QPointF(0, m_fieldWidth / 2 - 1));
    painter->drawLine(centerLineUp, flipFromOrigin(centerLineUp));

    // draw center cirlce
    QPointF circleUpleft = getOnImageCoordiante(QPointF(m_centerCircleDiameter / 2, -m_centerCircleDiameter / 2));
    QRectF centerCircle(circleUpleft, flipFromOrigin(circleUpleft));
    painter->drawEllipse(centerCircle);
    auto center = getOnImageCoordiante(QPointF(0, 0));
    QPointF x(center.x() - 2.5, center.y());
    QPointF y(center.x() + 2.5, center.y());
    painter->drawLine(x, y);

    // draw penalty mark
    QPointF leftPenalty = getOnImageCoordiante(QPointF(-m_fieldLength / 2 + m_penaltyMarkDistance, 0));
    drawCross(painter, leftPenalty);
    drawCross(painter, flipFromOrigin(leftPenalty));

}

void Field::drawCross(QPainter* painter, QPointF img) {
    // draw horizontal line
    {
        QPointF x(img.x() - 5, img.y());
        QPointF y(img.x() + 5, img.y());
        painter->drawRect(QRectF(x, y));
    }

    // draw vertical line
    {
        QPointF x(img.x(), img.y() - 5);
        QPointF y(img.x(), img.y() + 5);
        painter->drawRect(QRectF(x, y));
    }
}

QPointF Field::getOnImageCoordiante(QPointF real)
{
    QPointF res;
    res.setX(m_imageWidth / 2 + real.x());
    res.setY(m_imageHeight/ 2 - real.y());
    return res;
}

QPointF Field::getOnRealCoordinate(QPointF img) {
    QPointF res;
    res.setX(img.x() - m_imageWidth / 2);
    res.setY(-img.y() + m_imageHeight /2);
    return res;
}

QPointF Field::flipFromOrigin(QPointF p)
{
    auto real = getOnRealCoordinate(p);
    real.setX(-real.x());
    real.setY(-real.y());
    return getOnImageCoordiante(real);
}

QRectF Field::flipFromOrigin(QRectF rect)
{
   auto topLeft = rect.topLeft();
   auto bottomRight = rect.bottomRight();

   auto x = flipFromOrigin(topLeft);
   auto y = flipFromOrigin(bottomRight);

   return QRectF(y, x);
}




Field::~Field()
{

}

}
