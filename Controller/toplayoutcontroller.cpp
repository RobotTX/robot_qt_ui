#include "toplayoutcontroller.h"
#include <QMainWindow>
#include "Model/toplayout.h"
#include "View/toplayoutwidget.h"
#include "View/stylesettings.h"

TopLayoutController::TopLayoutController(QWidget *parent): QObject(parent)
{
    topLayout = QPointer<TopLayout> (new TopLayout());

    createTopLayoutView(parent);
}

void TopLayoutController::createTopLayoutView(QWidget* parent){
    view = new TopLayoutWidget(*topLayout, static_cast<QMainWindow*> (parent));
}

void TopLayoutController::removeRobotWithoutHome(const QString name){
    topLayout->removeRobotWithoutHome(name);
    view->setRobotNoHomeLabel(topLayout->getRobotsString());
}

void TopLayoutController::addRobotWithoutHome(const QString name){
    topLayout->addRobotWithoutHome(name);
    view->setRobotNoHomeLabel(topLayout->getRobotsString());
}

void TopLayoutController::setLabel(const QString msgType, const QString label){
    view->setLabel(msgType, label);
}

void TopLayoutController::setLabelDelay(const QString msgType, const QString label, int delay){
    view->setLabelDelay(msgType, label, delay);
}
