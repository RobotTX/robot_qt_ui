#include "displayselectedpointrobots.h"
#include "View/pointview.h"
#include <QLabel>
#include "Model/robots.h"
#include "Model/robot.h"
#include "View/robotview.h"
#include "Model/pathpoint.h"
#include <QLayoutItem>
#include "View/spacewidget.h"
#include <QButtonGroup>
#include "View/custompushbutton.h"
#include <QAbstractButton>

DisplaySelectedPointRobots::DisplaySelectedPointRobots(QWidget *parent):QWidget(parent){

    layout = new QVBoxLayout(this);

    homeWidget = new QWidget(this);
    QVBoxLayout* homeLayout = new QVBoxLayout(homeWidget);
    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    homeLayout->addWidget(spaceWidget);

    QLabel* homeLabel = new QLabel("This point is the home of the robot :", this);
    homeLabel->setWordWrap(true);
    homeLayout->addWidget(homeLabel);

    robotBtn = new CustomPushButton("Home", this);
    homeLayout->addWidget(robotBtn);
    connect(robotBtn, SIGNAL(clicked()), this, SLOT(robotBtnClicked()));

    homeWidget->hide();
    layout->addWidget(homeWidget);

    pathWidget = new QWidget(this);
    QVBoxLayout* pathLayout = new QVBoxLayout(pathWidget);

    QVBoxLayout* pathLabelLayout = new QVBoxLayout(pathWidget);
    SpaceWidget* spaceWidget2 = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    pathLabelLayout->addWidget(spaceWidget2);
    QLabel* pathLabel = new QLabel("This point belongs to the path of the robot(s) :", this);
    pathLabel->setWordWrap(true);
    pathLabelLayout->addWidget(pathLabel);
    pathLayout->addLayout(pathLabelLayout);

    pathBtnLayout = new QVBoxLayout();
    pathLayout->addLayout(pathBtnLayout);

    pathBtnGroup = new QButtonGroup(this);
    connect(pathBtnGroup, SIGNAL(buttonClicked(QAbstractButton*)), this, SLOT(pathBtnClicked(QAbstractButton*)));

    pathWidget->hide();
    layout->addWidget(pathWidget);

    homeLayout->setContentsMargins(0, 0, 0, 0);
    pathLayout->setContentsMargins(0, 0, 0, 0);
    layout->setContentsMargins(0, 0, 0, 0);
}

void DisplaySelectedPointRobots::setRobotsWidget(QSharedPointer<PointView> pointView, QSharedPointer<Robots> robots, const QString robotName){
    qDebug() << "DisplaySelectedPointRobots::setRobotsWidget called";
    removeAllPathButtons();

    if(pointView->getPoint()->isHome()){
        homeWidget->show();
        if(robotName.compare("") != 0)
            robotBtn->setText(robotName);
    } else {
        homeWidget->hide();
        robotBtn->setText("");
    }

    QSet<QString> robotNameSet;
    for(int i = 0; i < robots->getRobotsVector().size(); i++){
        QPointer<Robot> robot = robots->getRobotsVector().at(i)->getRobot();
        for(int j = 0; j < robot->getPath().size(); j++){
            if(robot->getPath().at(j)->getPoint().getName().compare(pointView->getPoint()->getName()) == 0)
                robotNameSet.insert(robot->getName());
        }
    }

    if(robotNameSet.count() > 0){
        QSetIterator<QString> k(robotNameSet);
        while (k.hasNext()){
            CustomPushButton* btn = new CustomPushButton(k.next(), this);
            pathBtnGroup->addButton(btn);
            pathBtnLayout->addWidget(btn);
        }
        pathWidget->show();
    } else
        pathWidget->hide();

    qDebug() << "DisplaySelectedPointRobots::setRobotsWidget yo" << pathBtnGroup->buttons().size() << pathBtnLayout->children().size();
}

void DisplaySelectedPointRobots::removeAllPathButtons(){
    qDebug() << "DisplaySelectedPointRobots::removeAllPathButtons called";

    QList<QAbstractButton*> list = pathBtnGroup->buttons();
    QListIterator<QAbstractButton*> i(list);
    while (i.hasNext())
        pathBtnGroup->removeButton(i.next());

    QLayoutItem *child;
    while ((child = pathBtnLayout->takeAt(0)) != 0) {
        delete child->widget();
        delete child;
    }

    qDebug() << "DisplaySelectedPointRobots::removeAllPathButtons yo" << pathBtnGroup->buttons().size() << pathBtnLayout->children().size();
}

void DisplaySelectedPointRobots::robotBtnClicked(){
    emit setSelectedRobotFromPoint(robotBtn->text());
}

void DisplaySelectedPointRobots::pathBtnClicked(QAbstractButton* button){
    emit setSelectedRobotFromPoint(button->text());
}

