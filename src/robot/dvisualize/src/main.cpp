// Created on: June 11, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QDebug>

int main(int argc, char *argv[]) {
    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine; 
    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));

    return app.exec();
}
