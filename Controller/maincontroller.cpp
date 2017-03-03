#include "maincontroller.h"
#include <QQmlProperty>
#include <QQmlApplicationEngine>
#include <QDebug>
#include <QVariant>
#include "Controller/mainmenucontroller.h"
#include "Controller/Map/mapcontroller.h"

MainController::MainController(QQmlApplicationEngine *engine, QObject* parent) : QObject(parent) {

    QList<QObject*> qmlList = engine->rootObjects();
    if(qmlList.size() == 1){
        /// The main parent element in the QML tree
        QObject *applicationWindow = qmlList.at(0);

        /// Main menu controller
        mainMenuController = new MainMenuController(applicationWindow, this);

        /// Map Controller
        mapController = new MapController(applicationWindow, this);
    } else {
        qDebug() << "MainController::MainController We are supposed to only have 1 item, the ApplicationWindow";
        Q_UNREACHABLE();
    }
}
