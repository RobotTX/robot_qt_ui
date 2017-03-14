#include "maincontroller.h"
#include <QQmlProperty>
#include <QQmlApplicationEngine>
#include <QDebug>
#include <QVariant>
#include <QImage>
#include "Controller/Map/mapcontroller.h"
#include "Controller/Point/pointcontroller.h"

MainController::MainController(QQmlApplicationEngine *engine, QObject* parent) : QObject(parent) {

    QList<QObject*> qmlList = engine->rootObjects();
    if(qmlList.size() == 1){
        /// The main parent element in the QML tree
        QObject *applicationWindow = qmlList.at(0);

        /// Map Controller
        mapController = new MapController(applicationWindow, this);

        /// Point Controller
        pointController = new PointController(applicationWindow, this);

    } else {
        qDebug() << "MainController::MainController We are supposed to only have 1 item, the ApplicationWindow";
        Q_UNREACHABLE();
    }
}

void MainController::checkPoint(QString name, double x, double y){
    /// When creating/editing a point we need the map to chekc if the point is on a wall or unknown place
    pointController->checkErrorPoint(mapController->getMapImage(), name, x, y);
}
