#include <QGuiApplication>
#include <QtQuick/QQuickView>
#include <QQmlApplicationEngine>
#include "Controller/maincontroller.h"
#include "Controller/authentification.h"
#include "View/EditMap/editmappainteditem.h"
#include "View/Robot/scanmappainteditem.h"
#include "View/Robot/obstaclespainteditem.h"

int main(int argc, char *argv[]) {

    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QGuiApplication app(argc, argv);

    qmlRegisterType<EditMapPaintedItem>("EditMapItems", 1, 0, "EditMapPaintedItem");
    qmlRegisterType<ScanMapPaintedItem>("ScanMapsPaintedItem", 1, 0, "ScanMapPaintedItem");
    qmlRegisterType<ObstaclesPaintedItem>("ObstaclesItems", 1, 0, "ObstaclesPaintedItems");

    // authentification window
    QQmlApplicationEngine auth;
    auth.load(QUrl("qrc:/auth.qml"));
//    qDebug("auth loading page");
    QQuickWindow *applicationWindowAuth = qobject_cast<QQuickWindow*>(auth.rootObjects().at(0));
    if (!applicationWindowAuth) {
       qFatal("Error: Your root item has to be a window.");
       return -1;
    }
    Authentification authen(&auth, applicationWindowAuth);

    return app.exec();
}
