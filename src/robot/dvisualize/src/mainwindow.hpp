#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCore>
#include "mymodel.hpp"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);

signals:

public slots:


private slots:
    void on_actionOpen_triggered();
    void on_currentChanged(QModelIndex current);

private:
    Ui::MainWindow* ui;
    MyModel* m_model;
};

#endif // MAINWINDOW_H
