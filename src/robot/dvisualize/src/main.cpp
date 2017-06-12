// Created on: June 11, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#include "mainwindow.hpp"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.setApp(&a);

    w.show();

    return a.exec();
}
