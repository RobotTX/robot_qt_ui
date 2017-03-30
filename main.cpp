#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include "Controller/maincontroller.h"
#include "View/editmappainteditem.h"
#include "View/mergemapspainteditem.h"
#include <QtQuick/QQuickView>

int main(int argc, char *argv[]) {
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QGuiApplication app(argc, argv);

    qmlRegisterType<EditMapPaintedItem>("EditMapItems", 1, 0, "EditMapPaintedItem");
    qmlRegisterType<MergeMapsPaintedItem>("MergeMapsItems", 1, 0, "MergeMapsPaintedItem");

    QQmlApplicationEngine engine;
    engine.load(QUrl("qrc:/main.qml"));

    QQuickWindow *applicationWindow = qobject_cast<QQuickWindow*>(engine.rootObjects().at(0));
    if (!applicationWindow) {
        qFatal("Error: Your root item has to be a window.");
        return -1;
    }

    MainController controller(&engine);

    return app.exec();
}
