#include <QDebug>
#include <QMouseEvent>
#include "undist.hpp"
#include "ui_undist.h"

Undist::Undist(QWidget *parent) : QDialog(parent), ui(new Ui::Undist) {
    ui->setupUi(this);
}

Undist::~Undist() {

}

void Undist::setPixmap(QPixmap img)
{
   m_currentImg = img;
   m_height = img.height();
   m_width = img.width();
   ui->img->setPixmap(img);
}

void Undist::onOriginalClicked(QMouseEvent *ev)
{
   m_x = ev->x();
   m_y = ev->y();
   qDebug() << "ori x: " << m_x << " y: " << m_y;

   update();
}


void Undist::update()
{
    // draw img;
    this->resize(m_currentImg.width(), m_currentImg.height());
    ui->img->resize(m_currentImg.width(), m_currentImg.height());


    // draw point !?
}
