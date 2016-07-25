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

    /// the widget is necessary to Qt, the scrollArea contains all widgets and is scrollable in the vertical direction
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

    /// to scroll a path when there are two many points to display
    pathScroll = new QScrollArea(this);
    pathScroll->setWidgetResizable(true);
    /*pathScroll->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    pathScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    pathScroll->setAlignment(Qt::AlignTop);
    pathScroll->setFrameShape(QFrame::NoFrame);*/

    /// The layout of the four columns
    QWidget* widgetName = new QWidget();
    QWidget* widgetPath = new QWidget(pathScroll);
    QWidget* widgetPlay = new QWidget();
    QWidget* widgetViewPath = new QWidget();
    QWidget* widgetStop = new QWidget();

    columnName = new QVBoxLayout();
    columnPath = new QVBoxLayout();
    columnPlay = new QVBoxLayout();
    columnViewPath = new QVBoxLayout();
    columnStop = new QVBoxLayout();

    widgetName->setLayout(columnName);
    widgetPlay->setLayout(columnPlay);
    widgetViewPath->setLayout(columnViewPath);
    widgetStop->setLayout(columnStop);

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
    //scrollLayout->addLayout(columnName);
    scrollLayout->addWidget(widgetName);

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
        columnPath->addWidget(pathLabel);
    }

    widgetPath->setLayout(columnPath);
    pathScroll->setWidget(widgetPath);
    scrollLayout->addWidget(pathScroll);
    scrollLayout->addLayout(columnPath);

    /// Creation of the third column, with the button to display the path the robot
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
    //scrollLayout->addLayout(columnViewPath);
    scrollLayout->addWidget(widgetViewPath);

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
    //scrollLayout->addLayout(columnPlay);
    scrollLayout->addWidget(widgetPlay);

    /// Creation of the fifth column, with the button to stop and delete the path of the robot
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
    scrollLayout->addWidget(widgetStop);

    /// We connect the groups of buttons to their respective slot in the main window
    connect(robotBtnGroup, SIGNAL(buttonClicked(QAbstractButton*)), parent, SLOT(setSelectedRobotNoParent(QAbstractButton*)));
    connect(stopRobotBtnGroup, SIGNAL(buttonClicked(int)), parent, SLOT(stopSelectedRobot(int)));
    connect(playRobotBtnGroup, SIGNAL(buttonClicked(int)), parent, SLOT(playSelectedRobot(int)));
    connect(viewPathRobotBtnGroup, SIGNAL(buttonToggled(int, bool)), parent, SLOT(viewPathSelectedRobot(int, bool)));

    setMaximumHeight(parent->height()*4/10);
    setMinimumHeight(parent->height()*4/10);

    widget->setLayout(scrollLayout);
    scrollArea->setWidget(widget);

    layout->addWidget(scrollArea);




    setStyleSheet("QWidget { border: 1px solid green; margin-top : 0px; }"
                  "QScrollArea { border: 1px solid #00BFFF; margin-top : 0px; }");
    widgetName->setStyleSheet("QWidget { border: 1px solid blue; margin-top : 0px; }");
    widgetPlay->setStyleSheet("QWidget { border: 1px solid violet; margin-top : 0px; }");
    widgetViewPath->setStyleSheet("QWidget { border: 1px solid yellow; margin-top : 0px; }");
    widgetStop->setStyleSheet("QWidget { border: 1px solid orange; margin-top : 0px; }");

    //widgetPath->setContentsMargins(0, 0, 0, 0);
    pathScroll->setStyleSheet("QWidget { border: 1px solid #FF69B4; margin-top : 0px; }"
                              "QScrollArea { border: 1px solid #4B0082; margin-top : 0px; }");


}

void BottomLayout::deletePath(const int index){
    /// When a path is deleted, the button to play/pause & stop the path are disabled
    /// and the path disappears from the list
    playRobotBtnGroup->button(index)->setEnabled(false);
    stopRobotBtnGroup->button(index)->setEnabled(false);
    viewPathRobotBtnGroup->button(index)->setEnabled(false);
    vectorPathLabel.at(index)->setText("");
}

void BottomLayout::updateRobot(const int id, RobotView * const robotView){
    //qDebug() << "(BottomLayout) updateRobot called" << id << robotBtnGroup->buttons().size();
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
    qDebug() << "(BottomLayout) addRobot called";
    int i = robotBtnGroup->buttons().size();
    /// Creation of the first column, with the button containing the name of the robots
    QPushButton* robotBtn = new QPushButton(robotView->getRobot()->getName(), this);
    robotBtn->setMinimumHeight(((QWidget*)parent())->height()/20);
    robotBtn->setMaximumHeight(((QWidget*)parent())->height()/20);
    robotBtn->setMaximumWidth(((QWidget*)parent())->width()*3/20);
    robotBtn->setMinimumWidth(((QWidget*)parent())->width()*3/20);
    robotBtnGroup->addButton(robotBtn, i);
    columnName->addWidget(robotBtn);

    /// Creation of the second column, with the labels containing the path of the robot
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

    /// Creation of the third column, with the button to display/stop displaying the robot
    QPushButton* viewPathRobotBtn = new QPushButton(QIcon(":/icons/eye.png"),"", this);
    viewPathRobotBtn->setMaximumWidth(((QWidget*)parent())->width()/20);
    viewPathRobotBtn->setMinimumWidth(((QWidget*)parent())->width()/20);
    viewPathRobotBtn->setIconSize(((QWidget*)parent())->size()/20);
    viewPathRobotBtn->setCheckable(true);
    if(robotView->getRobot()->getPath().size() < 1)
        viewPathRobotBtn->setEnabled(false);
    viewPathRobotBtnGroup->addButton(viewPathRobotBtn, i);
    columnViewPath->addWidget(viewPathRobotBtn);

    /// Creation of the fourth column, with the button to play/pause the robot
    QPushButton* playRobotBtn = new QPushButton(QIcon(":/icons/play.png"),"", this);
    playRobotBtn->setMaximumWidth(((QWidget*)parent())->width()/20);
    playRobotBtn->setMinimumWidth(((QWidget*)parent())->width()/20);
    playRobotBtn->setIconSize(((QWidget*)parent())->size()/20);
    if(robotView->getRobot()->getPath().size() < 1)
        playRobotBtn->setEnabled(false);
    playRobotBtnGroup->addButton(playRobotBtn, i);
    columnPlay->addWidget(playRobotBtn);

    /// Creation of the fifth column, with the button to stop and delete the path of the robot
    QPushButton* stopRobotBtn = new QPushButton(QIcon(":/icons/close.png"),"", this);
    stopRobotBtn->setMaximumWidth(((QWidget*)parent())->width()/20);
    stopRobotBtn->setMinimumWidth(((QWidget*)parent())->width()/20);
    stopRobotBtn->setIconSize(((QWidget*)parent())->size()/20);
    if(robotView->getRobot()->getPath().size() < 1)
        stopRobotBtn->setEnabled(false);
    stopRobotBtnGroup->addButton(stopRobotBtn, i);
    columnStop->addWidget(stopRobotBtn);
}

void BottomLayout::removeRobot(const int id){
    qDebug() << "(BottomLayout) removeRobot called" << id;
    if(id >= 0){
        playRobotBtnGroup->removeButton(playRobotBtnGroup->buttons().at(id));
        stopRobotBtnGroup->removeButton(stopRobotBtnGroup->buttons().at(id));
        robotBtnGroup->removeButton(robotBtnGroup->buttons().at(id));
        viewPathRobotBtnGroup->removeButton(viewPathRobotBtnGroup->buttons().at(id));
        vectorPathLabel.remove(id);

        QLayoutItem* item1 = columnName->takeAt(id);
        delete item1->widget();
        delete item1;
        QLayoutItem* item2 = columnPath->takeAt(id);
        delete item2->widget();
        delete item2;
        QLayoutItem* item3 = columnPlay->takeAt(id);
        delete item3->widget();
        delete item3;
        QLayoutItem* item4 = columnViewPath->takeAt(id);
        delete item4->widget();
        delete item4;
        QLayoutItem* item5 = columnStop->takeAt(id);
        delete item5->widget();
        delete item5;

        for(int i =0; i < playRobotBtnGroup->buttons().size(); i++){
            playRobotBtnGroup->setId(playRobotBtnGroup->buttons().at(i), i);
            stopRobotBtnGroup->setId(stopRobotBtnGroup->buttons().at(i), i);
            robotBtnGroup->setId(robotBtnGroup->buttons().at(i), i);
            viewPathRobotBtnGroup->setId(viewPathRobotBtnGroup->buttons().at(i), i);
        }
    } else {
        qDebug() << "(BottomLayout) Wrong id to remove" << id;
    }
}

void BottomLayout::setEnable(const bool enable){
    if(enable){

        for(int i =0; i < listEnabled.size(); i++){
            listEnabled.at(i)->setEnabled(true);
        }
        listEnabled.clear();

    } else {

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
}

void BottomLayout::uncheckViewPathSelectedRobot(const int robotNb){
    QList<QAbstractButton*> list = viewPathRobotBtnGroup->buttons();
    for(int i =0; i < list.size(); i++){
        if(list.at(i)->isChecked() && i != robotNb){
            list.at(i)->setChecked(false);
        }
    }
}


void BottomLayout::uncheckAll()
{
    QList<QAbstractButton*> list = viewPathRobotBtnGroup->buttons();
     for(int i =0; i < list.size(); i++){
         if(list.at(i)->isChecked()){
             list.at(i)->setChecked(false);
         }
     }
 }
