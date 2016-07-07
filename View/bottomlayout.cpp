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
#include <QDebug>

BottomLayout::BottomLayout(QMainWindow* parent, const std::shared_ptr<Robots> &robots) : QWidget(parent){
    layout = new QHBoxLayout(this);

    VerticalScrollArea* scrollArea = new VerticalScrollArea(this);
    /// We create a widget and a scroll area
    QWidget* widget = new QWidget(scrollArea);
    QHBoxLayout* scrollLayout = new QHBoxLayout(widget);

    QVector<RobotView*> robotsVector = robots->getRobotsVector();

    /// The button group for the collumn with the robots' name
    robotBtnGroup = new QButtonGroup(this);

    /// The button group for the collumn with the stop/delete path buttons
    viewPathRobotBtnGroup = new QButtonGroup(this);
    viewPathRobotBtnGroup->setExclusive(false);

    /// The button group for the collumn with the stop/delete path buttons
    stopRobotBtnGroup = new QButtonGroup(this);

    /// The button group for the collumn with the play/pause path buttons
    playRobotBtnGroup = new QButtonGroup(this);
    vectorPathLabel = QVector<QLabel*>();


    /*pathScroll = new QScrollArea(this);
    pathScroll->setWidgetResizable(true);
    pathScroll->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    pathScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    pathScroll->setFrameShape(QFrame::NoFrame);
    QWidget* widgetPath = new QWidget(pathScroll);
    widgetPath->setContentsMargins(0, 0, 0, 0);*/

    /// The layout of the four columns
    columnName = new QVBoxLayout();
    //QVBoxLayout* columnPath = new QVBoxLayout(widgetPath);
    columnPath = new QVBoxLayout();
    columnPlay = new QVBoxLayout();
    columnViewPath = new QVBoxLayout();
    columnStop = new QVBoxLayout();


    /// Creation of the first collumn, with the button containing the name of the robots
    for(int i = 0; i < robotsVector.size(); i++){
        QPushButton* robotBtn = new QPushButton(robotsVector.at(i)->getRobot()->getName(), this);
        robotBtn->setMinimumHeight(parent->height()/10);
        robotBtn->setMaximumHeight(parent->height()/10);
        robotBtn->setMaximumWidth(parent->width()*3/10);
        robotBtn->setMinimumWidth(parent->width()*3/10);
        robotBtnGroup->addButton(robotBtn, i);
        columnName->addWidget(robotBtn);
    }
    scrollLayout->addLayout(columnName);

    /// Creation of the second collumn, with the labels containing the path of the robot
    for(int i = 0; i < robotsVector.size(); i++){
        std::vector<std::shared_ptr<PathPoint>> path = robotsVector.at(i)->getRobot()->getPath();
        QString pathStr = QString("");
        for(size_t j = 0; j < path.size(); j++){
            if(j != 0){
                pathStr += " - ";
            }
            pathStr += path.at(j)->getPoint().getName();
        }
        QLabel* pathLabel = new QLabel(pathStr, this);
        pathLabel->setMinimumHeight(parent->height()/10);
        pathLabel->setMaximumHeight(parent->height()/10);
        vectorPathLabel.push_back(pathLabel);
        pathLabel->setMinimumWidth(1);
        columnPath->addWidget(pathLabel);
    }
    /*widgetPath->setLayout(columnPath);
    pathScroll->setWidget(widgetPath);
    scrollLayout->addWidget(pathScroll);*/
    scrollLayout->addLayout(columnPath);

    /// Creation of the third collumn, with the button to play/pause the robot
    for(int i = 0; i < robotsVector.size(); i++){
        QPushButton* viewPathRobotBtn = new QPushButton(QIcon(":/icons/eye.png"),"", this);
        viewPathRobotBtn->setMaximumWidth(parent->width()/10);
        viewPathRobotBtn->setMinimumWidth(parent->width()/10);
        viewPathRobotBtn->setIconSize(parent->size()/10);
        viewPathRobotBtn->setCheckable(true);
        if(robots->getRobotsVector().at(i)->getRobot()->getPath().size() < 1)
            viewPathRobotBtn->setEnabled(false);
        viewPathRobotBtnGroup->addButton(viewPathRobotBtn, i);
        columnViewPath->addWidget(viewPathRobotBtn);
    }
    scrollLayout->addLayout(columnViewPath);

    /// Creation of the fourth collumn, with the button to play/pause the robot
    for(int i = 0; i < robotsVector.size(); i++){
        QPushButton* playRobotBtn = new QPushButton(QIcon(":/icons/play.png"),"", this);
        playRobotBtn->setMaximumWidth(parent->width()/10);
        playRobotBtn->setMinimumWidth(parent->width()/10);
        playRobotBtn->setIconSize(parent->size()/10);
        if(robots->getRobotsVector().at(i)->getRobot()->getPath().size() < 1)
            playRobotBtn->setEnabled(false);
        playRobotBtnGroup->addButton(playRobotBtn, i);
        columnPlay->addWidget(playRobotBtn);
    }
    scrollLayout->addLayout(columnPlay);

    /// Creation of the fifth collumn, with the button to stop and delete the path of the robot
    for(int i = 0; i < robotsVector.size(); i++){
        QPushButton* stopRobotBtn = new QPushButton(QIcon(":/icons/close.png"),"", this);
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
    connect(robotBtnGroup, SIGNAL(buttonClicked(QAbstractButton*)), parent, SLOT(setSelectedRobotNoParent(QAbstractButton*)));
    connect(stopRobotBtnGroup, SIGNAL(buttonClicked(int)), parent, SLOT(stopSelectedRobot(int)));
    connect(playRobotBtnGroup, SIGNAL(buttonClicked(int)), parent, SLOT(playSelectedRobot(int)));
    connect(viewPathRobotBtnGroup, SIGNAL(buttonClicked(int)), parent, SLOT(viewPathSelectedRobot(int)));

    setMaximumHeight(parent->height()*4/10);
    setMinimumHeight(parent->height()*4/10);

    widget->setLayout(scrollLayout);
    scrollArea->setWidget(widget);

    layout->addWidget(scrollArea);
    //layout->setContentsMargins(0, 0, 0, 0);
}

void BottomLayout::deletePath(const int index){
    /// When a path is deleted, the button to play/pause & stop the path are disabled
    /// and the path disappear from the list
    playRobotBtnGroup->button(index)->setEnabled(false);
    stopRobotBtnGroup->button(index)->setEnabled(false);
    viewPathRobotBtnGroup->button(index)->setEnabled(false);
    vectorPathLabel.at(index)->setText("");
}

void BottomLayout::updateRobot(const int id, RobotView * const robotView){
    qDebug() << "(BottomLayout) updateRobot called";
    robotBtnGroup->button(id)->setText(robotView->getRobot()->getName());
    if(robotView->getRobot()->getPath().size() < 1){
        stopRobotBtnGroup->button(id)->setEnabled(false);
        playRobotBtnGroup->button(id)->setEnabled(false);
        viewPathRobotBtnGroup->button(id)->setEnabled(false);
        vectorPathLabel.at(id)->setText("");
     } else {
        stopRobotBtnGroup->button(id)->setEnabled(true);
        playRobotBtnGroup->button(id)->setEnabled(true);
        viewPathRobotBtnGroup->button(id)->setEnabled(true);
        QString pathStr = QString("");
        for(size_t j = 0; j < robotView->getRobot()->getPath().size(); j++){
            if(j != 0){
                pathStr += " - ";
            }
            pathStr += robotView->getRobot()->getPath().at(j)->getPoint().getName();
        }
        vectorPathLabel.at(id)->setText(pathStr);
    }
}

void BottomLayout::addRobot(RobotView * const robotView){
    int i = robotBtnGroup->buttons().size();
    /// Creation of the first collumn, with the button containing the name of the robots
    QPushButton* robotBtn = new QPushButton(robotView->getRobot()->getName(), this);
    robotBtn->setMinimumHeight(((QWidget*)parent())->height()/20);
    robotBtn->setMaximumHeight(((QWidget*)parent())->height()/20);
    robotBtn->setMaximumWidth(((QWidget*)parent())->width()*3/20);
    robotBtn->setMinimumWidth(((QWidget*)parent())->width()*3/20);
    robotBtnGroup->addButton(robotBtn, i);
    columnName->addWidget(robotBtn);

    /// Creation of the second collumn, with the labels containing the path of the robot
    std::vector<std::shared_ptr<PathPoint>> path = robotView->getRobot()->getPath();
    QString pathStr = QString("");
    for(size_t j = 0; j < path.size(); j++){
        if(j != 0){
            pathStr += " - ";
        }
        pathStr += path.at(j)->getPoint().getName();
    }
    QLabel* pathLabel = new QLabel(pathStr, this);
    pathLabel->setMinimumHeight(((QWidget*)parent())->height()/20);
    pathLabel->setMaximumHeight(((QWidget*)parent())->height()/20);
    vectorPathLabel.push_back(pathLabel);
    pathLabel->setMinimumWidth(1);
    columnPath->addWidget(pathLabel);

    /// Creation of the third collumn, with the button to play/pause the robot
    QPushButton* viewPathRobotBtn = new QPushButton(QIcon(":/icons/eye.png"),"", this);
    viewPathRobotBtn->setMaximumWidth(((QWidget*)parent())->width()/20);
    viewPathRobotBtn->setMinimumWidth(((QWidget*)parent())->width()/20);
    viewPathRobotBtn->setIconSize(((QWidget*)parent())->size()/20);
    viewPathRobotBtn->setCheckable(true);
    if(robotView->getRobot()->getPath().size() < 1)
        viewPathRobotBtn->setEnabled(false);
    viewPathRobotBtnGroup->addButton(viewPathRobotBtn, i);
    columnViewPath->addWidget(viewPathRobotBtn);

    /// Creation of the fourth collumn, with the button to play/pause the robot
    QPushButton* playRobotBtn = new QPushButton(QIcon(":/icons/play.png"),"", this);
    playRobotBtn->setMaximumWidth(((QWidget*)parent())->width()/20);
    playRobotBtn->setMinimumWidth(((QWidget*)parent())->width()/20);
    playRobotBtn->setIconSize(((QWidget*)parent())->size()/20);
    if(robotView->getRobot()->getPath().size() < 1)
        playRobotBtn->setEnabled(false);
    playRobotBtnGroup->addButton(playRobotBtn, i);
    columnPlay->addWidget(playRobotBtn);

    /// Creation of the fifth collumn, with the button to stop and delete the path of the robot
    QPushButton* stopRobotBtn = new QPushButton(QIcon(":/icons/close.png"),"", this);
    stopRobotBtn->setMaximumWidth(((QWidget*)parent())->width()/20);
    stopRobotBtn->setMinimumWidth(((QWidget*)parent())->width()/20);
    stopRobotBtn->setIconSize(((QWidget*)parent())->size()/20);
    if(robotView->getRobot()->getPath().size() < 1)
        stopRobotBtn->setEnabled(false);
    stopRobotBtnGroup->addButton(stopRobotBtn, i);
    columnStop->addWidget(stopRobotBtn);
}

/// TODO update all ids on delete
void BottomLayout::removeRobot(const int id){
    qDebug() << "removeRobot from BottomLayout soon" << id;
}

void BottomLayout::disable(){
    QList<QAbstractButton*> list = playRobotBtnGroup->buttons();
    for(int i =0; i < list.size(); i++){
        if(list.at(i)->isEnabled()){
            list.at(i)->setEnabled(false);
            listEnabled.push_back(list.at(i));
        }
    }

    list = stopRobotBtnGroup->buttons();
    for(int i =0; i < list.size(); i++){
        if(list.at(i)->isEnabled()){
            list.at(i)->setEnabled(false);
            listEnabled.push_back(list.at(i));
        }
    }

    list = robotBtnGroup->buttons();
    for(int i =0; i < list.size(); i++){
        if(list.at(i)->isEnabled()){
            list.at(i)->setEnabled(false);
            listEnabled.push_back(list.at(i));
        }
    }

    list = viewPathRobotBtnGroup->buttons();
    for(int i =0; i < list.size(); i++){
        if(list.at(i)->isEnabled()){
            list.at(i)->setEnabled(false);
            listEnabled.push_back(list.at(i));
        }
    }
}

void BottomLayout::enable(){
    for(int i =0; i < listEnabled.size(); i++){
        listEnabled.at(i)->setEnabled(true);
    }
    listEnabled.clear();
}

void BottomLayout::uncheckViewPathSelectedRobot(const int robotNb){
    QList<QAbstractButton*> list = viewPathRobotBtnGroup->buttons();
    for(int i =0; i < list.size(); i++){
        if(list.at(i)->isChecked() && i != robotNb){
            list.at(i)->setChecked(false);
        }
    }
}
