#include "clabel.hpp"
#include <QMouseEvent>
#include <QtCore>


CLabel::CLabel(QWidget *parent) : QLabel(parent)
{

}

void CLabel::mousePressEvent(QMouseEvent *ev)
{

}

void CLabel::mouseReleaseEvent(QMouseEvent *ev)
{
    mouseX = ev->x();
    mouseY = ev->y();

    emit clicked();
}

void CLabel::mouseMoveEvent(QMouseEvent *ev)
{

    mouseX = ev->x();
    mouseY = ev->y();

    emit clicked();
}

