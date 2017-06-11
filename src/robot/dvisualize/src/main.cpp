#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QDebug>

int main(int argc, char *argv[]) {
    QGuiApplication app(argc, argv);

    qDebug() << "fuck";
    QQmlApplicationEngine engine; 
    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));
    
    return app.exec();
}
