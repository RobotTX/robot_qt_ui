#include "bottomlayout.h"
#include "Model/robots.h"
#include "View/customscrollarea.h"
#include "View/robotview.h"
#include "Model/robot.h"
#include "Model/pathpoint.h"
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QMainWindow>
#include <QButtonGroup>
#include <QDebug>
#include <QScrollBar>
#include "Model/points.h"
#include "colors.h"

BottomLayout::BottomLayout(QMainWindow* parent, const QSharedPointer<Robots> &robots) : QWidget(parent), lastCheckedId(-1){

    layout = new QHBoxLayout(this);

    CustomScrollArea* scrollArea = new CustomScrollArea(this);

    /// the widget is necessary to Qt, the scrollArea contains all widgets and is scrollable in the vertical direction
    QWidget* widget = new QWidget(scrollArea);
    QHBoxLayout* scrollLayout = new QHBoxLayout(widget);

    QVector<RobotView*> robotsVector = robots->getRobotsVector();

    /// The button group for the column with the robots' name
    robotBtnGroup = new QButtonGroup(this);

    /// The button group for the column with the stop/delete path buttons
    viewPathRobotBtnGroup = new QButtonGroup(this);
    viewPathRobotBtnGroup->setExclusive(false);

    /// The button group for the column with the stop/delete path buttons
    stopRobotBtnGroup = new QButtonGroup(this);

    /// The button group for the column with the play/pause path buttons
    playRobotBtnGroup = new QButtonGroup(this);

    /// the button group for the column with the delete path buttons
    deletePathBtnGroup = new QButtonGroup(this);

    /// to scroll a path when there are two many points to display
    QScrollBar* pathScroll2 = new QScrollBar(Qt::Orientation::Horizontal, this);
    pathScroll = new CustomScrollArea(this, false, pathScroll2);

    /// The layout of the three columns
    widgetName = new QWidget(this);
    widgetPath = new QWidget(pathScroll);
    actionWidget = new QWidget(this);

    QHBoxLayout* actionLayout = new QHBoxLayout(actionWidget);

    columnName = new QVBoxLayout();
    columnPath = new QVBoxLayout();
    columnPlay = new QVBoxLayout();
    columnViewPath = new QVBoxLayout();
    columnStop = new QVBoxLayout();
    columnDelete = new QVBoxLayout();

    actionLayout->addLayout(columnViewPath);
    actionLayout->addLayout(columnPlay);
    actionLayout->addLayout(columnStop);
    actionLayout->addLayout(columnDelete);

    actionLayout->setContentsMargins(0,0,0,0);
    widgetName->setLayout(columnName);

    /// Creation of the first column, with the button containing the name of the robots
    for(int i = 0; i < robotsVector.size(); i++){
        QPushButton* robotBtn = new QPushButton(robotsVector.at(i)->getRobot()->getName(), this);
        robotBtn->setMinimumHeight(30);
        robotBtn->setMaximumHeight(30);
        robotBtn->setMaximumWidth(parent->width()*3/10);
        robotBtn->setMinimumWidth(parent->width()*3/10);
        robotBtnGroup->addButton(robotBtn, i);
        columnName->addWidget(robotBtn);
        robotBtn->setFlat(true);
        robotBtn->setCheckable(true);
    }

    scrollLayout->addWidget(widgetName);

    /// Creation of the second column, with the labels containing the path of the robot
    for(int i = 0; i < robotsVector.size(); i++){
        QLabel* pathLabel = new QLabel(pathToStr(robotsVector.at(i)->getRobot()->getPath(), robotsVector.at(i)->getLastStage()), this);
        pathLabel->setMinimumHeight(parent->height()/10);
        pathLabel->setMaximumHeight(parent->height()/10);
        vectorPathLabel.push_back(pathLabel);
        columnPath->addWidget(pathLabel);
    }

    widgetPath->setLayout(columnPath);
    pathScroll->setWidget(widgetPath);
    scrollLayout->addWidget(pathScroll);

    /// Creation of the third column, with the button to display the path the robot
    for(int i = 0; i < robotsVector.size(); i++){
        QPushButton* viewPathRobotBtn = new QPushButton(QIcon(":/icons/eye.png"),"", this);
        viewPathRobotBtn->setCheckable(true);
        viewPathRobotBtn->setMaximumWidth(parent->width()/10);
        viewPathRobotBtn->setMinimumWidth(parent->width()/10);
        viewPathRobotBtn->setIconSize(parent->size()/10);

        if(robots->getRobotsVector().at(i)->getRobot()->getPath().size() < 1)
            viewPathRobotBtn->setEnabled(false);
        viewPathRobotBtnGroup->addButton(viewPathRobotBtn, i);
        columnViewPath->addWidget(viewPathRobotBtn);
        viewPathRobotBtn->setFlat(true);
    }

    /// Creation of the fourth column, with the button to play/pause the robot
    for(int i = 0; i < robotsVector.size(); i++){
        QPushButton* playRobotBtn = new QPushButton(QIcon(":/icons/play.png"),"", this);
        if(robots->getRobotsVector().at(i)->getRobot()->getPath().size() < 1)
            playRobotBtn->setEnabled(false);
        playRobotBtnGroup->addButton(playRobotBtn, i);
        playRobotBtn->setMaximumWidth(parent->width()/10);
        playRobotBtn->setMinimumWidth(parent->width()/10);
        playRobotBtn->setIconSize(parent->size()/10);
        columnPlay->addWidget(playRobotBtn);
        playRobotBtn->setFlat(true);
    }

    ///  Creation of the fifth column to stop playing the path (new !)
    for(int i = 0; i < robotsVector.size(); i++){
        QPushButton* stopRobotBtn = new QPushButton(QIcon(":/icons/stop.png"),"", this);
        if(robots->getRobotsVector().at(i)->getRobot()->getPath().size() < 1)
            stopRobotBtn->setEnabled(false);
        stopRobotBtnGroup->addButton(stopRobotBtn, i);
        stopRobotBtn->setMaximumWidth(parent->width()/10);
        stopRobotBtn->setMinimumWidth(parent->width()/10);
        stopRobotBtn->setIconSize(parent->size()/10);
        columnStop->addWidget(stopRobotBtn);
        stopRobotBtn->setFlat(true);
    }

    /// Creation of the sixth column, with the button to delete the path of the robot
    for(int i = 0; i < robotsVector.size(); i++){
        QPushButton* deletePathButton = new QPushButton(QIcon(":/icons/bin.png"), "", this);
        if(robots->getRobotsVector().at(i)->getRobot()->getPath().size() < 1)
            deletePathButton->setEnabled(false);
        deletePathBtnGroup->addButton(deletePathButton, i);
        columnDelete->addWidget(deletePathButton);
        deletePathButton->setFlat(true);
        deletePathButton->setMaximumWidth(parent->width()/10);
        deletePathButton->setMinimumWidth(parent->width()/10);
        deletePathButton->setIconSize(parent->size()/10);
    }
    scrollLayout->addWidget(actionWidget);

    /// We connect the groups of buttons to their respective slot in the main window
    connect(robotBtnGroup, SIGNAL(buttonClicked(QAbstractButton*)), parent, SLOT(setSelectedRobotNoParent(QAbstractButton*)));
    connect(stopRobotBtnGroup, SIGNAL(buttonClicked(int)), parent, SLOT(stopPath(int)));
    connect(playRobotBtnGroup, SIGNAL(buttonClicked(int)), parent, SLOT(playSelectedRobot(int)));
    connect(viewPathRobotBtnGroup, SIGNAL(buttonToggled(int, bool)), parent, SLOT(viewPathSelectedRobot(int, bool)));
    connect(deletePathBtnGroup, SIGNAL(buttonClicked(int)), parent, SLOT(deletePath(int)));

    setMaximumHeight(parent->height()*4/10);
    setMinimumHeight(parent->height()*4/10);

    widget->setLayout(scrollLayout);
    scrollArea->setWidget(widget);

    layout->addWidget(scrollArea);


    pathScroll->horizontalScrollBar()->setStyleSheet(" * { color: transparent; "
                                                     "border: none; "
                                                     "background-color: transparent; "
                                                     "margin-top : 40px}");

    pathScroll2->move(pathScroll->pos().x() + 10, pathScroll->pos().y()
                      + pathScroll->height() - 45);
    pathScroll2->setMinimum(pathScroll->verticalScrollBar()->minimum());
    pathScroll2->setMaximum(pathScroll->verticalScrollBar()->maximum());
    pathScroll2->setValue(pathScroll->verticalScrollBar()->value());
    pathScroll2->setPageStep(pathScroll->verticalScrollBar()->pageStep());
    connect(pathScroll2, SIGNAL(valueChanged(int)), pathScroll->horizontalScrollBar(), SLOT(setValue(int)));
    pathScroll2->resize(pathScroll->width(), 15);
    actionLayout->setSpacing(0);

    QPalette pal;
    pal.setColor(QPalette::Background, bottom_menu_background_color);

    this->setPalette( pal);
    this->setAutoFillBackground(true);
}

void BottomLayout::updateRobot(const int id, RobotView * const robotView){
    qDebug() << "(BottomLayout) updateRobot called" << id << robotBtnGroup->buttons().size();
    robotBtnGroup->button(id)->setText(robotView->getRobot()->getName());
    if(robotView->getRobot()->getPath().size() < 1){
        deletePathBtnGroup->button(id)->setEnabled(false);
        playRobotBtnGroup->button(id)->setEnabled(false);
        viewPathRobotBtnGroup->button(id)->setEnabled(false);
        vectorPathLabel.at(id)->setText("");
     } else {
        deletePathBtnGroup->button(id)->setEnabled(true);
        playRobotBtnGroup->button(id)->setEnabled(true);
        viewPathRobotBtnGroup->button(id)->setEnabled(true);
        vectorPathLabel.at(id)->setText(pathToStr(robotView->getRobot()->getPath(), robotView->getLastStage()));
    }

    if(listEnabled.size() > 0){
        setEnable(true);
        setEnable(false);
    }
}

void BottomLayout::updateStageRobot(const int id, RobotView* robotView, const int stage){
    if(robotView->getRobot()->getPath().size() > 0){
        vectorPathLabel.at(id)->setText(pathToStr(robotView->getRobot()->getPath(), stage));
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
    QLabel* pathLabel = new QLabel(pathToStr(robotView->getRobot()->getPath(), robotView->getLastStage()), this);
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

    ///  Creation of the fifth column to stop playing the path (new !)
    QPushButton* stopRobotBtn = new QPushButton(QIcon(":/icons/stop.png"),"", this);
    stopRobotBtn->setMaximumWidth(((QWidget*)parent())->width()/20);
    stopRobotBtn->setMinimumWidth(((QWidget*)parent())->width()/20);
    stopRobotBtn->setIconSize(((QWidget*)parent())->size()/20);
    if(robotView->getRobot()->getPath().size() < 1)
        stopRobotBtn->setEnabled(false);

    stopRobotBtnGroup->addButton(stopRobotBtn, i);
    columnStop->addWidget(stopRobotBtn);

    /// Creation of the sixth column, with the button to delete the path of the robot
    QPushButton* deletePathButton = new QPushButton(QIcon(":/icons/close.png"),"", this);
    deletePathButton->setMaximumWidth(((QWidget*)parent())->width()/20);
    deletePathButton->setMinimumWidth(((QWidget*)parent())->width()/20);
    deletePathButton->setIconSize(((QWidget*)parent())->size()/20);
    if(robotView->getRobot()->getPath().size() < 1)
        deletePathButton->setEnabled(false);
    deletePathBtnGroup->addButton(deletePathButton, i);
    columnDelete->addWidget(deletePathButton);
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
        QLayoutItem* item6 = columnDelete->takeAt(id);
        delete item6->widget();
        delete item6;

        for(int i =0; i < playRobotBtnGroup->buttons().size(); i++){
            playRobotBtnGroup->setId(playRobotBtnGroup->buttons().at(i), i);
            stopRobotBtnGroup->setId(stopRobotBtnGroup->buttons().at(i), i);
            robotBtnGroup->setId(robotBtnGroup->buttons().at(i), i);
            viewPathRobotBtnGroup->setId(viewPathRobotBtnGroup->buttons().at(i), i);
            deletePathBtnGroup->setId(deletePathBtnGroup->buttons().at(i), i);
        }
    } else {
        qDebug() << "(BottomLayout) Wrong id to remove" << id;
    }
}

void BottomLayout::setEnable(const bool enable){
    qDebug() << "BottomLayout::setEnable called";
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

        list = deletePathBtnGroup->buttons();
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

void BottomLayout::uncheckAll(){
    QList<QAbstractButton*> list = viewPathRobotBtnGroup->buttons();
     for(int i =0; i < list.size(); i++){
         if(list.at(i)->isChecked()){
             list.at(i)->setChecked(false);
         }
     }
 }

QString BottomLayout::pathToStr(const QVector<QSharedPointer<PathPoint> >& path, const int stage){
    QString pathStr = QString("");
    for(int i = 0; i < path.size(); i++){
        if(stage > 0 && i < stage){
            pathStr += "<font color=\"green\">";
        }

        if(i != 0){
            pathStr += " - ";
        }

        if(path.at(i)->getPoint().getName().contains(PATH_POINT_NAME)){
            pathStr += QString::number(path.at(i)->getPoint().getPosition().getX(),'f', 1)
                    + "; "
                    + QString::number(path.at(i)->getPoint().getPosition().getY(),'f', 1);
        } else {
            pathStr += path.at(i)->getPoint().getName();
        }

        if(stage >0 && i <= stage){
            pathStr += "</font>";
        }
    }
    return pathStr;
}

void BottomLayout::uncheckRobots(){
    robotBtnGroup->setExclusive(false);
    if(robotBtnGroup->checkedButton())
        robotBtnGroup->checkedButton()->setChecked(false);
    robotBtnGroup->setExclusive(true);
}
