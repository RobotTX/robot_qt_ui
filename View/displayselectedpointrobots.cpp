#include "displayselectedpointrobots.h"
#include "View/pointview.h"
#include <QLabel>
#include "Model/robots.h"
#include "Model/robot.h"
#include "View/robotview.h"
#include "Model/pathpoint.h"
#include <QLayoutItem>
#include "View/spacewidget.h"

DisplaySelectedPointRobots::DisplaySelectedPointRobots(QWidget *parent):QWidget(parent){
    layout = new QVBoxLayout(this);

    homeWidget = new QWidget(this);
    QVBoxLayout* homeLayout = new QVBoxLayout(homeWidget);
    SpaceWidget* spaceWidget2 = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    homeLayout->addWidget(spaceWidget2);

    QLabel* homeLabel = new QLabel("This point is the home for the robot :", this);
    homeLabel->setWordWrap(true);
    homeLayout->addWidget(homeLabel);

    robotBtn = new QPushButton("", this);
    homeLayout->addWidget(robotBtn);
    connect(robotBtn, SIGNAL(clicked()), this, SLOT(robotBtnClicked()));

    homeWidget->hide();
    layout->addWidget(homeWidget);
}

void DisplaySelectedPointRobots::setRobotsWidget(PointView* pointView, std::shared_ptr<Robots> robots, const QString robotName){
    qDebug() << "DisplaySelectedPointRobots::setRobotsWidget called";
    /*removeAllItems(pathLayout);


    if(pointView->getPoint()->isHome()){
        homeWidget->show();
        if(robotName.compare("") != 0)
            robotBtn->setText(robotName);
    } else {
        homeWidget->hide();
        robotBtn->setText("");
    }
*/
    /*QSet<QString> robotNameSet;
    for(int i = 0; i < robots->getRobotsVector().size(); i++){
        std::shared_ptr<Robot> robot = robots->getRobotsVector().at(i)->getRobot();
        for(int j = 0; j < robot->getPath().size(); j++){
            if(robot->getPath().at(j)->getPoint().getName().compare(pointView->getPoint()->getName()) == 0){
                robotNameSet.insert(robot->getName());
            }
        }
    }

    qDebug() << "DisplaySelectedPoint::setRobotsLabel " << robotNameSet;
    QSetIterator<QString> k(robotNameSet);
    while (k.hasNext()){
        robotNameStr += k.next() + "\n";
    }*/
}

void DisplaySelectedPointRobots::removeAllItems(QLayout* _layout){
    QLayoutItem *child;
    while ((child = _layout->takeAt(0)) != 0) {
        delete child;
    }
}

void DisplaySelectedPointRobots::robotBtnClicked(){
    emit setSelectedRobotFromPoint(robotBtn->text());
}
