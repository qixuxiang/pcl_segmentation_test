#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCore>
#include <QApplication>
#include <QKeyEvent>
#include "mymodel.hpp"
#include "clabel.hpp"

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

    QVector<QPoint> m_realPoints;
    QApplication* m_app;
};

#endif // MAINWINDOW_H
