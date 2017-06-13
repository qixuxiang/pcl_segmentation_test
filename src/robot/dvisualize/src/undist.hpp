#pragma once
#include <QDialog>

namespace Ui {
   class Undist;
}

class Undist : public QDialog {
   Q_OBJECT

public:
    Undist(QWidget* parent);
    ~Undist();

    void setPixmap(QPixmap img);

 public slots:
    void onOriginalClicked(QMouseEvent* ev);
    void update();

private:
    Ui::Undist* ui;

    int m_height;
    int m_width;

    int m_x;
    int m_y;

    int m_undistx;
    int m_undisty;
    QPixmap m_currentImg;
};

