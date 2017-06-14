#pragma once
#include <QDialog>
#include <QPixmap>
#include "dvision/frame.hpp"
#include "dvision/distortionModel.hpp"
#include "mymodel.hpp"

namespace Ui {
   class Undist;
}

class Undist : public QDialog {
   Q_OBJECT

public:
    Undist(QWidget* parent);
    ~Undist();
    void setModel(MyModel*);

 public slots:
    void onOriginalClicked(QMouseEvent* ev);
    void updateView();

private:

    Ui::Undist* ui;

    int m_height;
    int m_width;

    int m_x = 0;
    int m_y = 0;

    int m_undistx;
    int m_undisty;
    QPixmap m_originImg;

    dvision::DistortionModel* m_distmodel;
    QPixmap m_undistImg;


    MyModel* m_listmodel;
};

