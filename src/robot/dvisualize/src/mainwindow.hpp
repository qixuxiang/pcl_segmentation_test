#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCore>
#include <QApplication>
#include <QKeyEvent>
#include "mymodel.hpp"
#include "clabel.hpp"
#include "dvision/parameters.hpp"
#include "dvision/distortionModel.hpp"
#include "undist.hpp"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);

    inline void setApp(QApplication* app) { m_app = app; }
private:
    void init();
    QPoint undistPoint(int x, int y);

signals:

public slots:


private slots:
    void on_actionOpen_triggered();
    void on_currentChanged(QModelIndex current);
    void keyReleaseEvent(QKeyEvent* ev) override;

    void appendText();
    void updateView();


    void on_actionSave_triggered();

private:
    Ui::MainWindow* ui;
    MyModel* m_model;
    CLabel* m_imgPanel;

    Undist* m_undist;

    QVector<QPoint> m_realPoints;
    QApplication* m_app;

    dvision::DistortionModel* m_distmodel;

    int m_pitch = -9999;
    int m_yaw = -9999;
};

#endif // MAINWINDOW_H
