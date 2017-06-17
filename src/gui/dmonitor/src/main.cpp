#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include "dmonitor/field.hpp"
#include "dmonitor/robot.hpp"

void registerType() {
    qmlRegisterType<dmonitor::Field>("DMonitor", 1, 0, "Field");
    qmlRegisterType<dmonitor::Robot>("DMonitor", 1, 0, "Robot");
}

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
    registerType();

    QQmlApplicationEngine engine;
    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));

    return app.exec();
}

