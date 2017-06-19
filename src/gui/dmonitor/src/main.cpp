#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include "dmonitor/field.hpp"
#include "dmonitor/robot.hpp"
#include "dmonitor/ball.hpp"
#include "dmonitor/guiImage.hpp"

void registerType() {
    qmlRegisterType<dmonitor::Field>("DMonitor", 1, 0, "Field");
    qmlRegisterType<dmonitor::Robot>("DMonitor", 1, 0, "Robot");
    qmlRegisterType<dmonitor::Ball>("DMonitor", 1, 0, "Ball");
    qmlRegisterType<dmonitor::GuiImage>("DMonitor", 1, 0, "GuiImage");
}

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
    registerType();

    QQmlApplicationEngine engine;
    engine.load(QUrl(QStringLiteral("qrc:/qml/main.qml")));

    return app.exec();
}

