#include "maincontroller.h"
#include <QQmlProperty>
#include <QQmlApplicationEngine>
#include <QDebug>
#include <QVariant>
#include "Controller/Menu/mainmenucontroller.h"
#include "Controller/Map/mapcontroller.h"
#include "Controller/Point/pointcontroller.h"

MainController::MainController(QQmlApplicationEngine *engine, QObject* parent) : QObject(parent) {

    QList<QObject*> qmlList = engine->rootObjects();
    if(qmlList.size() == 1){
        /// The main parent element in the QML tree
        QObject *applicationWindow = qmlList.at(0);

        /// Main menu controller
        mainMenuController = new MainMenuController(applicationWindow, this);

        /// Map Controller
        mapController = new MapController(applicationWindow, this);

        /// Point Controller
        pointController = new PointController(applicationWindow, mapController->getMapFile(), this);
    } else {
        qDebug() << "MainController::MainController We are supposed to only have 1 item, the ApplicationWindow";
        Q_UNREACHABLE();
    }
}
