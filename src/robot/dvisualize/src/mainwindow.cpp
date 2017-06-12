#include "mainwindow.hpp"
#include "ui_mainwindow.h"
#include <QtCore>
#include <QFileDialog>
#include <QListView>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow), m_model(new MyModel(this))
{
    ui->setupUi(this);

    auto& view = ui->listView;
    view->setModel(m_model);
    view->setViewMode(QListView::IconMode);

    connect(view->selectionModel(), &QItemSelectionModel::currentChanged,
            this, &MainWindow::on_currentChanged);

    // debug
    m_model->loadImages("/home/mwx/Pictures/1495350747129219434.png");
}


void MainWindow::on_actionOpen_triggered()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Open Image"), "/home/", tr("Image files (*.png *.jpg)"));
    m_model->loadImages(filename);
}

void MainWindow::on_currentChanged(QModelIndex current)
{
//    Q_UNUSED(previous);
    qDebug() << current.row();
}
