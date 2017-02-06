#include "bottomlayout.h"
#include "Model/robots.h"
#include "View/customscrollarea.h"
#include "View/robotview.h"
#include "Model/robot.h"
#include "Model/pathpoint.h"
#include <QMainWindow>
#include <QDebug>
#include "Model/points.h"
#include "View/stylesettings.h"
#include "View/custompushbutton.h"
#include <QButtonGroup>
#include <QLabel>
#include <QHBoxLayout>
#include <QAbstractButton>
#include <QScrollBar>
#include <QVBoxLayout>

BottomLayout::BottomLayout(QMainWindow* parent, const QSharedPointer<Robots> &robots) : QWidget(parent), lastCheckedId(-1){

    layout = new QHBoxLayout(this);

    CustomScrollArea* scrollArea = new CustomScrollArea(this);

    /// the widget is necessary to Qt, the scrollArea contains all widgets and is scrollable in the vertical direction
    QWidget* widget = new QWidget(scrollArea);
    QHBoxLayout* scrollLayout = new QHBoxLayout(widget);

    QVector<QPointer<RobotView>> robotsVector = robots->getRobotsVector();

    /// The button group for the column with the robots' name
    robotBtnGroup = new QButtonGroup(this);

    /// The button group for the column with the stop/delete path buttons
    viewPathRobotBtnGroup = new QButtonGroup(this);
    viewPathRobotBtnGroup->setExclusive(false);

    /// The button group for the column with the stop/delete path buttons
    stopRobotBtnGroup = new QButtonGroup(this);

    homeBtnGroup = new QButtonGroup(this);

    /// The button group for the column with the play/pause path buttons
    playRobotBtnGroup = new QButtonGroup(this);

    /// the button group for the column with the delete path buttons
    deletePathBtnGroup = new QButtonGroup(this);

    /// to scroll a path when there are two many points to display
    QScrollBar* pathScroll2 = new QScrollBar(Qt::Orientation::Horizontal, this);
    QWidget* hidingWidget = new QWidget(this);
    pathScroll = new CustomScrollArea(this, false, false, false, pathScroll2, hidingWidget);

    /// The layout of the three columns
    widgetName = new QWidget(this);
    widgetPath = new QWidget(pathScroll);
    actionWidget = new QWidget(this);

    QHBoxLayout* actionLayout = new QHBoxLayout(actionWidget);

    columnName = new QVBoxLayout();
    columnPath = new QVBoxLayout();
    columnHome = new QVBoxLayout();
    columnPlay = new QVBoxLayout();
    columnViewPath = new QVBoxLayout();
    columnStop = new QVBoxLayout();
    columnDelete = new QVBoxLayout();

    actionLayout->addLayout(columnViewPath);
    actionLayout->addLayout(columnHome);
    actionLayout->addLayout(columnPlay);
    actionLayout->addLayout(columnStop);
    actionLayout->addLayout(columnDelete);

    widgetName->setLayout(columnName);

    /// Creation of the first column, with the button containing the name of the robots
    for(int i = 0; i < robotsVector.size(); i++){
        CustomPushButton* robotBtn = new CustomPushButton(robotsVector.at(i)->getRobot()->getName(), this, CustomPushButton::ButtonType::BOTTOM, "left", true);
        robotBtn->setMaximumWidth(parent->width()*3/10);
        robotBtn->setMinimumWidth(parent->width()*3/10);
        robotBtnGroup->addButton(robotBtn, i);
        columnName->addWidget(robotBtn);
    }

    scrollLayout->addWidget(widgetName);

    /// Creation of the second column, with the labels containing the path of the robot
    for(int i = 0; i < robotsVector.size(); i++){
        QLabel* pathLabel = new QLabel(pathToStr(robotsVector.at(i)->getRobot()->getPath(), robotsVector.at(i)->getLastStage()), this);
        pathLabel->setMinimumHeight(m_button_height);
        pathLabel->setMaximumHeight(m_button_height);
        vectorPathLabel.push_back(pathLabel);
        columnPath->addWidget(pathLabel);
    }

    widgetPath->setLayout(columnPath);
    pathScroll->setWidget(widgetPath);
    scrollLayout->addWidget(pathScroll);

    /// Creation of the third column, with the button to display the path the robot
    for(int i = 0; i < robotsVector.size(); i++){
        CustomPushButton* viewPathRobotBtn = new CustomPushButton(QIcon(":/icons/eye.png"),"", this, CustomPushButton::ButtonType::BOTTOM, "left", true);
        viewPathRobotBtn->setMaximumWidth(parent->width()/10);
        viewPathRobotBtn->setMinimumWidth(parent->width()/10);
        viewPathRobotBtn->setIconSize(s_icon_size);
        viewPathRobotBtn->setToolTip("Click to display the path");
        if(robots->getRobotsVector().at(i)->getRobot()->getPath().size() < 1)
            viewPathRobotBtn->setEnabled(false);
        viewPathRobotBtnGroup->addButton(viewPathRobotBtn, i);
        columnViewPath->addWidget(viewPathRobotBtn);
    }

    /// Creation of the fourth column to send robots home
    for(int i = 0; i < robotsVector.size(); i++){
        CustomPushButton* goHomeBtn = new CustomPushButton(QIcon(":/icons/home.png"), "", this, CustomPushButton::ButtonType::BOTTOM, "left", true);
        goHomeBtn->setMaximumWidth(parent->width()/10);
        goHomeBtn->setMinimumWidth(parent->width()/10);
        goHomeBtn->setIconSize(s_icon_size);
        goHomeBtn->setToolTip("Click to go home");
        homeBtnGroup->addButton(goHomeBtn, i);
        columnHome->addWidget(goHomeBtn);
        goHomeBtn->setCheckable(false);
    }

    /// Creation of the fifth column, with the button to play/pause the robot
    for(int i = 0; i < robotsVector.size(); i++){
        CustomPushButton* playRobotBtn = new CustomPushButton(QIcon(":/icons/play.png"),"", this, CustomPushButton::ButtonType::BOTTOM);
        if(robots->getRobotsVector().at(i)->getRobot()->getPath().size() < 1)
            playRobotBtn->setEnabled(false);
        playRobotBtn->setToolTip("Click to play/pause the robot");
        playRobotBtnGroup->addButton(playRobotBtn, i);
        playRobotBtn->setMaximumWidth(parent->width()/10);
        playRobotBtn->setMinimumWidth(parent->width()/10);
        playRobotBtn->setIconSize(xs_icon_size);
        columnPlay->addWidget(playRobotBtn);
    }

    ///  Creation of the sixth column to stop playing the path (new !)
    for(int i = 0; i < robotsVector.size(); i++){
        CustomPushButton* stopRobotBtn = new CustomPushButton(QIcon(":/icons/stop.png"),"", this, CustomPushButton::ButtonType::BOTTOM);
        stopRobotBtn->setEnabled(false);
        stopRobotBtn->setToolTip("Click to stop the robot");
        stopRobotBtnGroup->addButton(stopRobotBtn, i);
        stopRobotBtn->setMaximumWidth(parent->width()/10);
        stopRobotBtn->setMinimumWidth(parent->width()/10);
        stopRobotBtn->setIconSize(xs_icon_size);
        columnStop->addWidget(stopRobotBtn);
    }

    /// Creation of the seventh column, with the button to delete the path of the robot
    for(int i = 0; i < robotsVector.size(); i++){
        CustomPushButton* deletePathButton = new CustomPushButton(QIcon(":/icons/empty.png"), "", this, CustomPushButton::ButtonType::BOTTOM);
        if(robots->getRobotsVector().at(i)->getRobot()->getPath().size() < 1)
            deletePathButton->setEnabled(false);
        deletePathButton->setToolTip("Click to delete the path");
        deletePathBtnGroup->addButton(deletePathButton, i);
        columnDelete->addWidget(deletePathButton);
        deletePathButton->setMaximumWidth(parent->width()/10);
        deletePathButton->setMinimumWidth(parent->width()/10);
        deletePathButton->setIconSize(s_icon_size);
    }

    scrollLayout->addWidget(actionWidget);

    /// We connect the groups of buttons to their respective slot in the main window
    connect(robotBtnGroup, SIGNAL(buttonClicked(QAbstractButton*)), parent, SLOT(setSelectedRobotNoParent(QAbstractButton*)));
    connect(homeBtnGroup, SIGNAL(buttonClicked(int)), parent, SLOT(goHome(int)));
    connect(stopRobotBtnGroup, SIGNAL(buttonClicked(int)), parent, SLOT(stopPath(int)));
    connect(playRobotBtnGroup, SIGNAL(buttonClicked(int)), parent, SLOT(playSelectedRobot(int)));
    connect(viewPathRobotBtnGroup, SIGNAL(buttonToggled(int, bool)), parent, SLOT(viewPathSelectedRobot(int, bool)));
    connect(deletePathBtnGroup, SIGNAL(buttonClicked(int)), parent, SLOT(deletePath(int)));

    setMaximumHeight(parent->height()*4/10);
    setMinimumHeight(parent->height()*4/10);

    widget->setLayout(scrollLayout);
    scrollArea->setWidget(widget);

    layout->addWidget(scrollArea);

    pathScroll2->move(pathScroll->pos().x() + 9, pathScroll->pos().y()
                      + pathScroll->height() - 41);
    pathScroll2->setMinimum(pathScroll->verticalScrollBar()->minimum());
    pathScroll2->setMaximum(pathScroll->verticalScrollBar()->maximum());
    pathScroll2->setValue(pathScroll->verticalScrollBar()->value());
    pathScroll2->setPageStep(pathScroll->verticalScrollBar()->pageStep());
    connect(pathScroll2, SIGNAL(valueChanged(int)), pathScroll->horizontalScrollBar(), SLOT(setValue(int)));

    hidingWidget->move(pathScroll->pos().x() + 9, pathScroll->pos().y()
                 + pathScroll->height() - 26);
    hidingWidget->setStyleSheet("* { background-color: " + bottom_menu_background_color + "; }");

    QPalette pal;
    pal.setColor(QPalette::Background, bottom_menu_background_color);

    this->setPalette(pal);
    this->setAutoFillBackground(true);
}

void BottomLayout::updateRobot(const int id, QPointer<RobotView> const robotView){
    qDebug() << "(BottomLayout) updateRobot called" << id << robotBtnGroup->buttons().size();
    static_cast<CustomPushButton*> (robotBtnGroup->button(id))->setText(robotView->getRobot()->getName());

    updateStageRobot(id, robotView, robotView->getLastStage());

    if(robotView->getRobot()->getPath().size() > 0){
        deletePathBtnGroup->button(id)->setEnabled(true);
        playRobotBtnGroup->button(id)->setEnabled(true);
        viewPathRobotBtnGroup->button(id)->setEnabled(true);
        if(abs(robotView->getLastStage()) > 0 || robotView->getRobot()->isPlayingPath())
            stopRobotBtnGroup->button(id)->setEnabled(true);
        else
            stopRobotBtnGroup->button(id)->setEnabled(false);
    } else {
        deletePathBtnGroup->button(id)->setEnabled(false);
        playRobotBtnGroup->button(id)->setEnabled(false);
        viewPathRobotBtnGroup->button(id)->setEnabled(false);
        stopRobotBtnGroup->button(id)->setEnabled(false);
    }

    homeBtnGroup->button(id)->setEnabled(true);

    if(listEnabled.size() > 0){
        setEnable(true);
        setEnable(false);
    }
}

void BottomLayout::updateStageRobot(const int id, QPointer<RobotView> robotView, const int stage){
    if(robotView->getRobot()->getPath().size() > 0)
        vectorPathLabel.at(id)->setText(pathToStr(robotView->getRobot()->getPath(), stage));
    else
        vectorPathLabel.at(id)->setText("");

}

void BottomLayout::addRobot(QPointer<RobotView> const robotView){
    qDebug() << "(BottomLayout) addRobot called";
    int i = robotBtnGroup->buttons().size();
    /// Creation of the first column, with the button containing the name of the robots
    CustomPushButton* robotBtn = new CustomPushButton(robotView->getRobot()->getName(), this, CustomPushButton::ButtonType::BOTTOM, "left", true);
    robotBtn->setMaximumWidth(static_cast<QWidget*>(parent())->width()*3/20);
    robotBtn->setMinimumWidth(static_cast<QWidget*>(parent())->width()*3/20);
    robotBtnGroup->addButton(robotBtn, i);
    columnName->addWidget(robotBtn);

    /// Creation of the second column, with the labels containing the path of the robot
    QLabel* pathLabel = new QLabel(pathToStr(robotView->getRobot()->getPath(), robotView->getLastStage()), this);
    pathLabel->setMinimumHeight(m_button_height);
    pathLabel->setMaximumHeight(m_button_height);
    vectorPathLabel.push_back(pathLabel);
    pathLabel->setMinimumWidth(1);
    columnPath->addWidget(pathLabel);

    /// Creation of the third column, with the button to display/stop displaying the robot
    CustomPushButton* viewPathRobotBtn = new CustomPushButton(QIcon(":/icons/eye.png"),"", this, CustomPushButton::ButtonType::BOTTOM, "left", true);
    viewPathRobotBtn->setMaximumWidth(static_cast<QWidget*>(parent())->width()/20);
    viewPathRobotBtn->setMinimumWidth(static_cast<QWidget*>(parent())->width()/20);
    viewPathRobotBtn->setIconSize(s_icon_size);
    if(robotView->getRobot()->getPath().size() < 1)
        viewPathRobotBtn->setEnabled(false);
    viewPathRobotBtnGroup->addButton(viewPathRobotBtn, i);
    columnViewPath->addWidget(viewPathRobotBtn);

    /// Creation of the fourth column with the button to send the robot home
    CustomPushButton* goHomeBtn = new CustomPushButton(QIcon(":/icons/home.png"), "", this, CustomPushButton::ButtonType::BOTTOM, "left", true);
    goHomeBtn->setMaximumWidth(static_cast<QWidget*>(parent())->width()/20);
    goHomeBtn->setMinimumWidth(static_cast<QWidget*>(parent())->width()/20);
    goHomeBtn->setCheckable(false);
    homeBtnGroup->addButton(goHomeBtn, i);
    columnHome->addWidget(goHomeBtn);

    /// Creation of the fifth column, with the button to play/pause the robot
    CustomPushButton* playRobotBtn = new CustomPushButton(QIcon(":/icons/play.png"),"", this, CustomPushButton::ButtonType::BOTTOM);
    playRobotBtn->setMaximumWidth(((QWidget*)parent())->width()/20);
    playRobotBtn->setMinimumWidth(((QWidget*)parent())->width()/20);
    playRobotBtn->setIconSize(xs_icon_size);
    if(robotView->getRobot()->getPath().size() < 1)
        playRobotBtn->setEnabled(false);
    playRobotBtnGroup->addButton(playRobotBtn, i);
    columnPlay->addWidget(playRobotBtn);

    ///  Creation of the sixth column to stop playing the path (new !)
    CustomPushButton* stopRobotBtn = new CustomPushButton(QIcon(":/icons/stop.png"),"", this, CustomPushButton::ButtonType::BOTTOM);
    stopRobotBtn->setMaximumWidth(static_cast<QWidget*>(parent())->width()/20);
    stopRobotBtn->setMinimumWidth(static_cast<QWidget*>(parent())->width()/20);
    stopRobotBtn->setIconSize(xs_icon_size);

    stopRobotBtn->setEnabled(false);

    stopRobotBtnGroup->addButton(stopRobotBtn, i);
    columnStop->addWidget(stopRobotBtn);

    /// Creation of the seventh column, with the button to delete the path of the robot
    CustomPushButton* deletePathButton = new CustomPushButton(QIcon(":/icons/empty.png"),"", this, CustomPushButton::ButtonType::BOTTOM);
    deletePathButton->setMaximumWidth(static_cast<QWidget*>(parent())->width()/20);
    deletePathButton->setMinimumWidth(static_cast<QWidget*>(parent())->width()/20);
    deletePathButton->setIconSize(s_icon_size);
    if(robotView->getRobot()->getPath().size() < 1)
        deletePathButton->setEnabled(false);
    deletePathBtnGroup->addButton(deletePathButton, i);
    columnDelete->addWidget(deletePathButton);
}

void BottomLayout::removeRobot(const int id){
    qDebug() << "(BottomLayout) removeRobot called" << id;
    if(id >= 0){
        if(lastCheckedId == id)
            lastCheckedId = -1;
        else if(lastCheckedId > id)
            lastCheckedId--;

        for(int i =0; i < listEnabled.size(); i++){
            if(listEnabled.at(i) == playRobotBtnGroup->buttons().at(id)
                    || listEnabled.at(i) == stopRobotBtnGroup->buttons().at(id)
                    || listEnabled.at(i) == robotBtnGroup->buttons().at(id)
                    || listEnabled.at(i) == viewPathRobotBtnGroup->buttons().at(id)
                    || listEnabled.at(i) == homeBtnGroup->buttons().at(id))
                listEnabled.removeAt(i);
        }

        playRobotBtnGroup->removeButton(playRobotBtnGroup->buttons().at(id));
        stopRobotBtnGroup->removeButton(stopRobotBtnGroup->buttons().at(id));
        robotBtnGroup->removeButton(robotBtnGroup->buttons().at(id));
        viewPathRobotBtnGroup->removeButton(viewPathRobotBtnGroup->buttons().at(id));
        homeBtnGroup->removeButton(homeBtnGroup->buttons().at(id));
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
        QLayoutItem* item7 = columnHome->takeAt(id);
        delete item7->widget();
        delete item7;

        for(int i = 0; i < playRobotBtnGroup->buttons().size(); i++){
            playRobotBtnGroup->setId(playRobotBtnGroup->buttons().at(i), i);
            stopRobotBtnGroup->setId(stopRobotBtnGroup->buttons().at(i), i);
            robotBtnGroup->setId(robotBtnGroup->buttons().at(i), i);
            viewPathRobotBtnGroup->setId(viewPathRobotBtnGroup->buttons().at(i), i);
            deletePathBtnGroup->setId(deletePathBtnGroup->buttons().at(i), i);
            homeBtnGroup->setId(homeBtnGroup->buttons().at(i), i);
        }
    } else {
        qDebug() << "(BottomLayout) Wrong id to remove" << id;
    }
}

void BottomLayout::setEnable(const bool enable){
    //qDebug() << "BottomLayout::setEnable called";
    if(enable){
        for(int i = 0; i < listEnabled.size(); i++){
            if(listEnabled.at(i))
                listEnabled.at(i)->setEnabled(true);
        }
        listEnabled.clear();
    } else {
        QList<QAbstractButton*> list = playRobotBtnGroup->buttons();
        for(int i = 0; i < list.size(); i++){
            if(list.at(i)->isEnabled()){
                list.at(i)->setEnabled(false);
                listEnabled.push_back(list.at(i));
            }
        }

        list = stopRobotBtnGroup->buttons();
        for(int i = 0; i < list.size(); i++){
            if(list.at(i)->isEnabled()){
                list.at(i)->setEnabled(false);
                listEnabled.push_back(list.at(i));
            }
        }

        list = robotBtnGroup->buttons();
        for(int i = 0; i < list.size(); i++){
            if(list.at(i)->isEnabled()){
                list.at(i)->setEnabled(false);
                listEnabled.push_back(list.at(i));
            }
        }

        list = viewPathRobotBtnGroup->buttons();
        for(int i = 0; i < list.size(); i++){
            if(list.at(i)->isEnabled()){
                list.at(i)->setEnabled(false);
                listEnabled.push_back(list.at(i));
            }
        }

        list = deletePathBtnGroup->buttons();
        for(int i = 0; i < list.size(); i++){
            if(list.at(i)->isEnabled()){
                list.at(i)->setEnabled(false);
                listEnabled.push_back(list.at(i));
            }
        }
        list = homeBtnGroup->buttons();
        for(int i = 0; i < list.size(); i++){
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
        if(list.at(i)->isChecked() && i != robotNb)
            list.at(i)->setChecked(false);
    }
}

void BottomLayout::uncheckAll(){
    QList<QAbstractButton*> list = viewPathRobotBtnGroup->buttons();
     for(int i =0; i < list.size(); i++){
         if(list.at(i)->isChecked())
             list.at(i)->setChecked(false);
     }
 }

QString BottomLayout::pathToStr(const QVector<QSharedPointer<PathPoint> >& path, const int stage){
    QString pathStr = QString("");
    for(int i = 0; i < path.size(); i++){
        if(i < abs(stage)){
            if(stage > 0)
                pathStr += "<font color=\"green\">";
            else if(stage < 0)
                pathStr += "<font color=\"red\">";
        }

        if(i != 0)
            pathStr += " - ";

        if(path.at(i)->getPoint().getName().contains(PATH_POINT_NAME))
            pathStr += QString::number(path.at(i)->getPoint().getPosition().getX(),'f', 1)
                    + "; "
                    + QString::number(path.at(i)->getPoint().getPosition().getY(),'f', 1);
        else
            pathStr += path.at(i)->getPoint().getName();

        if(stage != 0 && i <= abs(stage))
            pathStr += "</font>";
    }
    return pathStr;
}

void BottomLayout::uncheckRobots(){
    robotBtnGroup->setExclusive(false);
    if(robotBtnGroup->checkedButton())
        robotBtnGroup->checkedButton()->setChecked(false);
    robotBtnGroup->setExclusive(true);
}
