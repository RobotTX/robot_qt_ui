#include "mainmenucontroller.h"
#include <QDebug>

MainMenuController::MainMenuController(QObject *applicationWindow, QObject *parent) : QObject(parent){
    QObject *mainMenuFrame = applicationWindow->findChild<QObject*>("mainMenuFrame");
    if (mainMenuFrame){
        connect(mainMenuFrame, SIGNAL(selectMenu(QString, bool)), this, SLOT(menuClicked(QString,bool)));
        connect(this, SIGNAL(closeMenu(QVariant)), mainMenuFrame, SLOT(closeMenu(QVariant)));
    } else {
        qDebug() << "MainController::MainController could not find the main menu frame";
        Q_UNREACHABLE();
    }

    QObject *mainMenuViewsFrame = applicationWindow->findChild<QObject*>("mainMenuViewsFrame");
    if (mainMenuViewsFrame){
        connect(this, SIGNAL(showMenu(QVariant)), mainMenuViewsFrame, SLOT(showMenu(QVariant)));
    } else {
        qDebug() << "MainController::MainController could not find the mainMenuViews frame";
        Q_UNREACHABLE();
    }

    QObject *robotMenuHeader = applicationWindow->findChild<QObject*>("robotMenuHeader");
    if (robotMenuHeader){
        connect(robotMenuHeader, SIGNAL(closeMenu(QString)), this, SLOT(closeMenuClicked(QString)));
    } else {
        qDebug() << "MainController::MainController could not find the robot menu header";
        Q_UNREACHABLE();
    }

    QObject *pathMenuHeader = applicationWindow->findChild<QObject*>("pathMenuHeader");
    if (pathMenuHeader){
        connect(pathMenuHeader, SIGNAL(closeMenu(QString)), this, SLOT(closeMenuClicked(QString)));
    } else {
        qDebug() << "MainController::MainController could not find the path menu header";
        Q_UNREACHABLE();
    }

    QObject *pointMenuHeader = applicationWindow->findChild<QObject*>("pointMenuHeader");
    if (pointMenuHeader){
        connect(pointMenuHeader, SIGNAL(closeMenu(QString)), this, SLOT(closeMenuClicked(QString)));
    } else {
        qDebug() << "MainController::MainController could not find the point menu header";
        Q_UNREACHABLE();
    }

    QObject *mapMenuHeader = applicationWindow->findChild<QObject*>("mapMenuHeader");
    if (mapMenuHeader){
        connect(mapMenuHeader, SIGNAL(closeMenu(QString)), this, SLOT(closeMenuClicked(QString)));
    } else {
        qDebug() << "MainController::MainController could not find the map menu header";
        Q_UNREACHABLE();
    }

    QObject *settingsMenuHeader = applicationWindow->findChild<QObject*>("settingsMenuHeader");
    if (settingsMenuHeader){
        connect(settingsMenuHeader, SIGNAL(closeMenu(QString)), this, SLOT(closeMenuClicked(QString)));
    } else {
        qDebug() << "MainController::MainController could not find the settings menu header";
        Q_UNREACHABLE();
    }
}

void MainMenuController::menuClicked(QString txt, bool checked){
    qDebug() << "MainController::menuClicked" << txt << checked;
    /// NOTE do stuff like update the menu
    emit showMenu(txt);
}

void MainMenuController::closeMenuClicked(QString txt){
    qDebug() << "MainController::closeMenuClicked" << txt;
    emit closeMenu(txt);
}


