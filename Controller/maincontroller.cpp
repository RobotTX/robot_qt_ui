#include "maincontroller.h"
#include <QQmlProperty>
#include <QQmlApplicationEngine>
#include <QDebug>
#include "Controller/mainmenucontroller.h"

MainController::MainController(QQmlApplicationEngine *_engine, QObject* parent) : QObject(parent), engine(_engine) {
    /// Main menu controller
    mainMenuController = new MainMenuController(this);

    QList<QObject*> qmlList = engine->rootObjects();
    if(qmlList.size() == 1){
        QObject *mainMenuFrame = qmlList.at(0)->findChild<QObject*>("mainMenuFrame");
        if (mainMenuFrame){
            qDebug() << "MainController::MainController Yeah got the main menu frame";
            connect(mainMenuFrame, SIGNAL(selectMenu(QString, bool)), this, SLOT(menuClicked(QString,bool)));
        } else {
            qDebug() << "MainController::MainController could not find the main menu frame";
            Q_UNREACHABLE();
        }
    } else {
        qDebug() << "MainController::MainController We are supposed to only have 1 item, the ApplicationWindow";
        Q_UNREACHABLE();
    }
}

void MainController::menuClicked(QString txt, bool checked){
    qDebug() << "MainController::menuClicked" << txt << checked;
    /// NOTE do stuff like update the menu
    QList<QObject*> qmlList = engine->rootObjects();
    if(qmlList.size() == 1){
        QObject *mainMenuViewsFrame = qmlList.at(0)->findChild<QObject*>("mainMenuViewsFrame");
        if (mainMenuViewsFrame){
            QVariant returnedValue;
            qDebug() << "MainController::MainController Yeah got the mainMenuViewsframe";
            QMetaObject::invokeMethod(mainMenuViewsFrame, "showMenu", Q_RETURN_ARG(QVariant, returnedValue), Q_ARG(QVariant, QVariant::fromValue(txt)));
        } else {
            qDebug() << "MainController::MainController could not find the mainMenuViews frame";
            Q_UNREACHABLE();
        }
    }
}
