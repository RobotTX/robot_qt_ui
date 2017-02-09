#include "toplayoutcontroller.h"
#include <QMainWindow>
#include "View/toplayoutwidget.h"
#include "View/stylesettings.h"

TopLayoutController::TopLayoutController(QWidget *parent): QObject(parent)
{
    createTopLayoutView(parent);
}

void TopLayoutController::createTopLayoutView(QWidget* parent){
    view = new TopLayoutWidget(static_cast<QMainWindow*> (parent));
}

void TopLayoutController::removeRobotWithoutHome(const QString name){
    for(int i = 0; i < robotsWithoutHome.size(); i++){
        if(!robotsWithoutHome.at(i).compare(name)){
            robotsWithoutHome.remove(i);
            return;
        }
    }
    /// updates the view reflecting the last robot being removed
    view->setRobotNoHomeLabel(getRobotsString());
}

void TopLayoutController::addRobotWithoutHome(const QString name){
    robotsWithoutHome.push_back(name);
    /// updates the view reflecting the last robot being added
    view->setRobotNoHomeLabel(getRobotsString());
}

void TopLayoutController::setLabel(const QString msgType, const QString label){
    view->setLabel(msgType, label);
}

void TopLayoutController::setLabelDelay(const QString msgType, const QString label, int delay){
    view->setLabelDelay(msgType, label, delay);
}

QString TopLayoutController::getRobotsString() const {
    QString robots_string("");
    for(int i = 0; i < robotsWithoutHome.size(); i++){
        if(!robots_string.isEmpty())
            robots_string += ", ";
        robots_string += robotsWithoutHome.at(i);
    }
    return robots_string;
}
