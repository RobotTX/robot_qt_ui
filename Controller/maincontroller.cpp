#include "maincontroller.h"
#include <QQmlProperty>
#include <QQmlApplicationEngine>
#include <QDebug>
#include <QVariant>
#include "Controller/mainmenucontroller.h"

MainController::MainController(QQmlApplicationEngine *engine, QObject* parent) : QObject(parent) {
    /// Main menu controller
    MainMenuController* mainMenuController = new MainMenuController(engine, this);
}
