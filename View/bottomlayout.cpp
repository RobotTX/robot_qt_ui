#include "bottomlayout.h"
#include "Model/robots.h"
#include "View/verticalscrollarea.h"
#include "View/robotview.h"
#include "Model/robot.h"
#include "Model/pathpoint.h"
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QMainWindow>
#include <QButtonGroup>

BottomLayout::BottomLayout(QMainWindow* parent, Robots* const& robots){
    layout = new QHBoxLayout();

    /// We create a widget and a scroll area
    QWidget* widget = new QWidget();
    QHBoxLayout* scrollLayout = new QHBoxLayout();
    VerticalScrollArea* scrollArea = new VerticalScrollArea(this);

    QVector<RobotView*> robotsVector = robots->getRobotsVector();

    /// The button group for the collumn with the robots' name
    robotBtnGroup = new QButtonGroup();

    /// The button group for the collumn with the stop/delete path buttons
    stopRobotBtnGroup = new QButtonGroup();

    /// The button group for the collumn with the play/pause path buttons
    playRobotBtnGroup = new QButtonGroup();
    vectorPathLabel = QVector<QLabel*>();

    /// The layout of the four columns
    QVBoxLayout* columnName = new QVBoxLayout();
    QVBoxLayout* columnPath = new QVBoxLayout();
    QVBoxLayout* columnPlay = new QVBoxLayout();
    QVBoxLayout* columnStop = new QVBoxLayout();


    /// Creation of the first collumn, with the button containing the name of the robots
    for(int i = 0; i < robotsVector.size(); i++){
        QPushButton* robotBtn = new QPushButton(robotsVector.at(i)->getRobot()->getName());
        robotBtn->setMaximumWidth(parent->width()*3/10);
        robotBtn->setMinimumWidth(parent->width()*3/10);
        robotBtnGroup->addButton(robotBtn, i);
        columnName->addWidget(robotBtn);
    }
    scrollLayout->addLayout(columnName);

    /// Creation of the second collumn, with the labels containing the path of the robot
    for(int i = 0; i < robotsVector.size(); i++){
        QVector<PathPoint*> path = robotsVector.at(i)->getRobot()->getPath();
        QString pathStr = QString("");
        for(int j = 0; j < path.size(); j++){
            if(j != 0){
                pathStr += "; ";
            }
            pathStr += path.at(j)->getPoint().getName();
        }
        QLabel* pathLabel = new QLabel(pathStr);
        vectorPathLabel.push_back(pathLabel);
        pathLabel->setMinimumWidth(1);
        columnPath->addWidget(pathLabel);
    }
    scrollLayout->addLayout(columnPath);

    /// Creation of the third collumn, with the button to play/pause the robot
    QIcon icon = QIcon(":/icons/play.png");
    icon.setThemeName("play");
    for(int i = 0; i < robotsVector.size(); i++){
        QPushButton* playRobotBtn = new QPushButton(icon,"");
        playRobotBtn->setMaximumWidth(parent->width()/10);
        playRobotBtn->setMinimumWidth(parent->width()/10);
        playRobotBtn->setIconSize(parent->size()/10);
        if(robots->getRobotsVector().at(i)->getRobot()->getPath().size() < 1)
            playRobotBtn->setEnabled(false);
        playRobotBtnGroup->addButton(playRobotBtn, i);
        columnPlay->addWidget(playRobotBtn);
    }
    scrollLayout->addLayout(columnPlay);

    /// Creation of the first collumn, with the button to stop and delete the path of the robot
    for(int i = 0; i < robotsVector.size(); i++){
        QPushButton* stopRobotBtn = new QPushButton(QIcon(":/icons/close.png"),"");
        stopRobotBtn->setMaximumWidth(parent->width()/10);
        stopRobotBtn->setMinimumWidth(parent->width()/10);
        stopRobotBtn->setIconSize(parent->size()/10);
        if(robots->getRobotsVector().at(i)->getRobot()->getPath().size() < 1)
            stopRobotBtn->setEnabled(false);
        stopRobotBtnGroup->addButton(stopRobotBtn, i);
        columnStop->addWidget(stopRobotBtn);
    }
    scrollLayout->addLayout(columnStop);

    /// We connect the groups of buttons to their respective slot in the main window
    connect(robotBtnGroup, SIGNAL(buttonClicked(QAbstractButton*)), parent, SLOT(setSelectedRobot(QAbstractButton*)));
    connect(stopRobotBtnGroup, SIGNAL(buttonClicked(int)), parent, SLOT(stopSelectedRobot(int)));
    connect(playRobotBtnGroup, SIGNAL(buttonClicked(int)), parent, SLOT(playSelectedRobot(int)));

    setMaximumHeight(parent->height()*4/10);
    setMinimumHeight(parent->height()*4/10);

    widget->setLayout(scrollLayout);
    scrollArea->setWidget(widget);

    layout->addWidget(scrollArea);

    setLayout(layout);
}

BottomLayout::~BottomLayout(){
    delete layout;
    delete playRobotBtnGroup;
    delete stopRobotBtnGroup;
    delete robotBtnGroup;
}

void BottomLayout::deletePath(const int index){
    /// When a path is deleted, the button to play/pause & stop the path are disabled
    /// and the path disappear from the list
    playRobotBtnGroup->button(index)->setEnabled(false);
    stopRobotBtnGroup->button(index)->setEnabled(false);
    vectorPathLabel.at(index)->setText("");
}

void BottomLayout::updateRobot(const int id, RobotView * const robotView){
    robotBtnGroup->button(id)->setText(robotView->getRobot()->getName());
    if(robotView->getRobot()->getPath().size() < 1){
        stopRobotBtnGroup->button(id)->setEnabled(false);
        playRobotBtnGroup->button(id)->setEnabled(false);
        vectorPathLabel.at(id)->setText("");
     } else {
        stopRobotBtnGroup->button(id)->setEnabled(true);
        playRobotBtnGroup->button(id)->setEnabled(true);
        QString pathStr = QString("");
        for(int j = 0; j < robotView->getRobot()->getPath().size(); j++){
            if(j != 0){
                pathStr += " - ";
            }
            pathStr += robotView->getRobot()->getPath().at(j)->getPoint().getName();
        }
        vectorPathLabel.at(id)->setText(pathStr);
    }
}
