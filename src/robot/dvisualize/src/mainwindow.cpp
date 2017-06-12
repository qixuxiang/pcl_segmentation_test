#include "mainwindow.hpp"
#include "ui_mainwindow.h"
#include "clabel.hpp"
#include <QtCore>
#include <QFileDialog>
#include <QListView>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow), m_model(new MyModel(this))
{
    ui->setupUi(this);
    init();
    ui->listView->setModel(m_model);
    ui->listView->setViewMode(QListView::IconMode);

    connect(ui->listView->selectionModel(), &QItemSelectionModel::currentChanged,
            this, &MainWindow::on_currentChanged);

    connect(ui->imagePanel, &CLabel::clicked,
            this, &MainWindow::updateView);

    // debug
    m_model->loadImages("/home/mwx/Pictures/calibration/1496034577976886624.png");
}

void MainWindow::init()
{
    m_imgPanel = ui->imagePanel;

    m_realPoints << QPoint(10, 10);
    m_realPoints << QPoint(10, -10);

    m_realPoints << QPoint(20, 10);
    m_realPoints << QPoint(20, -10);

    m_realPoints << QPoint(75, 0); // circle front
    m_realPoints << QPoint(0, -75);// circle right
    m_realPoints << QPoint(0, 75); // circle left

    m_realPoints << QPoint(240, 0); // penalty point


    m_realPoints << QPoint(0, 300); // T left
    m_realPoints << QPoint(0, -300); // T right

    m_realPoints << QPoint(450, 130); // left goal
    m_realPoints << QPoint(450, -130); // right goal

    m_realPoints << QPoint(450, 300); // left corner
    m_realPoints << QPoint(450, -300); // right corner


    auto* box = ui->candidateReal;
    foreach(const QPoint& p, m_realPoints){
       box->addItem(QString("%1, %2").arg(p.x()).arg(p.y()));
    }
}

QPoint MainWindow::undistPoint(int x, int y)
{
   return QPoint(x, y);
}




void MainWindow::on_currentChanged(QModelIndex current)
{
    m_model->setCurrentIndex(current.row());
    ui->imagePanel->setPixmap(m_model->getImage());
    updateView();
//    auto size = ui->imagePanel->pixmap()->size();
    //    ui->imagePanel->setFixedSize(size.width(), size.height());
}



void MainWindow::keyReleaseEvent(QKeyEvent *ev)
{
    if(ev->isAutoRepeat())
        return;

    if(ev->key() == Qt::Key_Space) {
        appendText();
    } else if (ev->key() == Qt::Key_Escape) {
//        qDebug() << "Exit";
//        m_app->closeAllWindows();
    }
}

void MainWindow::appendText()
{
    auto* textEdit = ui->textEdit;
    auto real = m_realPoints.at(ui->candidateReal->currentIndex());

    // !? todo, calc undistorted img position
    int x = m_imgPanel->x();
    int y = m_imgPanel->y();
    auto p = undistPoint(x, y);
    x = p.x();
    y = p.y();

    textEdit->append(QString("%1 %2 %3 %4")
            .arg(x)
            .arg(y)
            .arg(real.x())
            .arg(real.y()));
}

void MainWindow::updateView()
{
    QString imgpos = QString("%1, %2").arg(m_imgPanel->x()).arg(m_imgPanel->y());
    ui->labelImagePosition->setText(imgpos);
}

void MainWindow::on_actionSave_triggered()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
                               QDir::homePath(),
                               tr("Plain text (*.txt)"));

    qDebug() << fileName << endl;

    if(fileName == NULL)
        return;

    QFile file(fileName);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return;
    }

    QTextStream in(&file);
    in << ui->textEdit->toPlainText();
    file.close();

//    m_app->closeAllWindows();
}

void MainWindow::on_actionOpen_triggered()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Open Image"), "/home/", tr("Image files (*.png *.jpg)"));
    if(filename != NULL) {
        m_model->loadImages(filename);
        ui->listView->setCurrentIndex(m_model->index(0, 0));
    }
}
