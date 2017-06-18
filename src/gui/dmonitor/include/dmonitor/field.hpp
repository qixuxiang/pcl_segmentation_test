#pragma once
#include <QQuickPaintedItem>

// TODO(MWX): field config

namespace dmonitor {
class Field : public QQuickPaintedItem {
    Q_OBJECT
public:
    Field(QQuickItem* parent = 0);
    ~Field();
    void paint(QPainter *painter) override;
    void drawField(QPainter* painter);

    // convert point's coordinate from real(center of the field) to image (up left corner) for drawing
    QPointF getOnImageCoordiante(QPointF real);
    QPointF getOnRealCoordinate(QPointF img);
    QPointF flipFromOrigin(QPointF p);
    QRectF flipFromOrigin(QRectF rect);

    void drawCross(QPainter *painter, QPointF real);
private:
    // Field geometry in cm, refer to RoboCup Humanoid Rules
    int m_fieldLength = 900;
    int m_fieldWidth = 600;
    int m_goalDepth = 60;
    int m_goalWidth = 260;
    int m_goalHeight = 180;
    int m_goalAreaLength = 100;
    int m_goalAreaWidth = 500;
    int m_penaltyMarkDistance = 210;
    int m_centerCircleDiameter = 150;
    int m_borderStripWidth = 70;
    int m_lineWidth = 5;

    int m_imageWidth = m_fieldLength + m_borderStripWidth * 2;
    int m_imageHeight = m_fieldWidth + m_borderStripWidth * 2;

    // color
    QColor m_grassGreen = QColor(1, 142, 14);
};
}
