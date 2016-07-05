#include "leftmenu.h"
#include "View/editselectedrobotwidget.h"
#include "View/editselectedpointwidget.h"
#include "View/pointview.h"
#include "View/bottomlayout.h"
#include "View/leftmenuwidget.h"
#include "View/pointsleftwidget.h"
#include "View/selectedrobotwidget.h"
#include "View/robotsleftwidget.h"
#include "View/mapleftwidget.h"
#include "View/pointsview.h"
#include "View/displayselectedpoint.h"
#include "View/displayselectedgroup.h"
#include "View/pathcreationwidget.h"
#include "Controller/mainwindow.h"
#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QDebug>
#include "View/pointbuttongroup.h"
#include <QScrollArea>
#include "verticalscrollarea.h"
#include <QButtonGroup>
#include "Model/group.h"
#include "Model/points.h"
#include "Model/xmlparser.h"
#include "Controller/mainwindow.h"

LeftMenu::LeftMenu(MainWindow* _parent, std::shared_ptr<Points> const& _points, const std::shared_ptr<Robots> &robots, PointsView * const &pointViews):
    QWidget(_parent), parent(_parent), points(_points)
{

    QVBoxLayout * leftLayout  = new QVBoxLayout();

    QVBoxLayout * globalLayout  = new QVBoxLayout(this);
    QHBoxLayout * topLayout  = new QHBoxLayout();

    returnButton = new QPushButton(QIcon(":/icons/arrowLeft.png"), " Return", this);
    returnButton->setIconSize(_parent->size()/10);
    connect(returnButton, SIGNAL(clicked()), _parent, SLOT(backEvent()));

    closeBtn = new QPushButton(QIcon(":/icons/cropped_close.png"), "", this);
    closeBtn->setIconSize(_parent->size()/30);
    closeBtn->setFlat(true);
    //closeBtn->setStyleSheet("QPushButton { padding: 5px;}");
    //closeBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    returnButton->hide();
    topLayout->addWidget(returnButton);

    topLayout->addWidget(closeBtn);

    leftLayout->addLayout(topLayout);
    connect(closeBtn, SIGNAL(clicked()), _parent, SLOT(closeSlot()));

    /// to display the information relative to a point
    displaySelectedPoint = new DisplaySelectedPoint(_parent, _points);
    leftLayout->addWidget(displaySelectedPoint);
    leftLayout->setAlignment(displaySelectedPoint, Qt::AlignLeft);

    /// to display the information relative to a group of points
    displaySelectedGroup = new DisplaySelectedGroup(_parent, _points);
    leftLayout->addWidget(displaySelectedGroup);
    leftLayout->setAlignment(displaySelectedGroup, Qt::AlignLeft);

    /// The first menu with 3 buttons : Robots, Points, Map
    leftMenuWidget = new LeftMenuWidget(_parent);
    leftLayout->addWidget(leftMenuWidget);

    /// Menu which display the list of points
    pointsLeftWidget = new PointsLeftWidget(_parent, _points);
    leftLayout->addWidget(pointsLeftWidget);

    /// Menu which display the selected robot infos
    selectedRobotWidget = new SelectedRobotWidget(_parent);
    connect(selectedRobotWidget, SIGNAL(selectHome(RobotView*)), _parent, SLOT(selectHomeEvent()));
    connect(selectedRobotWidget, SIGNAL(showSelectedRobotWidget()), _parent, SLOT(showHome()));
    connect(selectedRobotWidget, SIGNAL(hideSelectedRobotWidget()), _parent, SLOT(hideHome()));
    leftLayout->addWidget(selectedRobotWidget);

    /// Menu which display the list of robots
    robotsLeftWidget = new RobotsLeftWidget(_parent);
    robotsLeftWidget->setRobots(robots);
    leftLayout->addWidget(robotsLeftWidget);

    /// Menu which display the map menu in which user can save/load a map
    mapLeftWidget = new MapLeftWidget(_parent);
    leftLayout->addWidget(mapLeftWidget);

    /// Menu to edit the selected robot
    editSelectedRobotWidget = new EditSelectedRobotWidget(_parent, robots);
    leftLayout->addWidget(editSelectedRobotWidget);
    connect(editSelectedRobotWidget, SIGNAL(robotSaved()), _parent, SLOT(robotSavedEvent()));
    connect(editSelectedRobotWidget, SIGNAL(showEditSelectedRobotWidget()), _parent, SLOT(showHome()));
    connect(editSelectedRobotWidget, SIGNAL(hideEditSelectedRobotWidget()), _parent, SLOT(hideHome()));

    /// Menu to edit the selected point
    editSelectedPointWidget = new EditSelectedPointWidget(_parent, pointViews);
    leftLayout->addWidget(editSelectedPointWidget);
    connect(editSelectedPointWidget, SIGNAL(pointSaved(int, double, double, QString)), _parent, SLOT(pointSavedEvent(int, double, double, QString)));

    /// Menu which display the widget for the creation of a path
    pathCreationWidget = new PathCreationWidget(_parent, *_points);
    leftLayout->addWidget(pathCreationWidget);


    connect(pathCreationWidget, SIGNAL(updatePathPointToPainter(QVector<Point>*)), _parent, SLOT(updatePathPointToPainter(QVector<Point>*)));
    connect(pathCreationWidget, SIGNAL(hidePathCreationWidget()), _parent, SLOT(hidePathCreationWidget()));
    connect(pathCreationWidget, SIGNAL(editTmpPathPoint(int, Point*, int)), _parent, SLOT(editTmpPathPointSlot(int, Point*, int)));
    connect(pathCreationWidget, SIGNAL(saveEditPathPoint()), _parent, SLOT(saveTmpEditPathPointSlot()));
    connect(pathCreationWidget, SIGNAL(setMessage(QString, QString)), _parent, SLOT(setMessageTop(QString, QString)));

    connect(displaySelectedPoint->getRobotButton(), SIGNAL(clicked()), _parent, SLOT(setSelectedRobotFromPoint()));
    connect(displaySelectedPoint->getActionButtons()->getMinusButton(), SIGNAL(clicked(bool)), _parent, SLOT(removePointFromInformationMenu()));
    connect(displaySelectedPoint->getActionButtons()->getMapButton(), SIGNAL(clicked(bool)), _parent, SLOT(displayPointMapEvent()));
    connect(displaySelectedPoint->getActionButtons()->getEditButton(), SIGNAL(clicked(bool)), _parent, SLOT(editPointButtonEvent()));

    // to try maybe later connect(displaySelectedGroup->getMinusButton(), SIGNAL(clicked(bool)), this, SLOT(removePoint()));
    connect(displaySelectedGroup->getActionButtons()->getMinusButton(), SIGNAL(clicked(bool)), _parent, SLOT(removePointFromGroupMenu()));
    connect(displaySelectedGroup->getActionButtons()->getEditButton(), SIGNAL(clicked(bool)), _parent, SLOT(editPointFromGroupMenu()));
    connect(displaySelectedGroup->getActionButtons()->getEyeButton(), SIGNAL(clicked(bool)), _parent, SLOT(displayPointInfoFromGroupMenu()));
    connect(displaySelectedGroup->getActionButtons()->getMapButton(), SIGNAL(clicked(bool)), _parent, SLOT(displayPointFromGroupMenu()));

    /// to enable the buttons
    connect(displaySelectedGroup->getPointButtonGroup()->getButtonGroup(), SIGNAL(buttonClicked(int)), this, SLOT(enableButtons(int)));

    hide();
    leftLayout->setContentsMargins(0,0,0,0);
    setMaximumWidth(_parent->width()*4/10);
    setMinimumWidth(_parent->width()*4/10);
    leftLayout->setAlignment(Qt::AlignTop);
    leftLayout->setAlignment(closeBtn, Qt::AlignTop | Qt::AlignRight);

   for (int i = 0; i < leftLayout->count(); ++i) {
        QWidget *widget = leftLayout->itemAt(i)->widget();
        if (widget != NULL) {
            widget->setMinimumWidth(1);
        }
    }

    globalLayout->addLayout(leftLayout);
    globalLayout->setContentsMargins(0, 0, 0, 0);
}

void LeftMenu::updateGroupDisplayed(std::shared_ptr<Points> const& _points, const int groupIndex){
    displaySelectedGroup->getPointButtonGroup()->setGroup(_points, groupIndex);
}

void LeftMenu::hideBackButton(void)
{
    if(returnButton != NULL)
        returnButton->hide();
}

void LeftMenu::showBackButton(QString name)
{
    if(returnButton){
        returnButton->setText(name);
        returnButton->show();
    }
}

void LeftMenu::enableButtons(int index){
    displaySelectedGroup->getActionButtons()->getMapButton()->setCheckable(true);
    /// enables the minus button
    displaySelectedGroup->getActionButtons()->getMinusButton()->setEnabled(true);
    displaySelectedGroup->getActionButtons()->getMinusButton()->setToolTip("Click to remove the selected point");
    /// enables the eye button
    displaySelectedGroup->getActionButtons()->getEyeButton()->setEnabled(true);
    displaySelectedGroup->getActionButtons()->getEyeButton()->setToolTip("Click to see the information of the selected point");
    /// enables the map button
    displaySelectedGroup->getActionButtons()->getMapButton()->setEnabled(true);
    if(displaySelectedGroup->getPoints()->getGroups().at(displaySelectedGroup->getPointButtonGroup()->getGroupIndex())->getPoints().at(index)->isDisplayed()){
        displaySelectedGroup->getActionButtons()->getMapButton()->setChecked(true);
        displaySelectedGroup->getActionButtons()->getMapButton()->setToolTip("Click to hide the selected point on the map");
    } else {
        displaySelectedGroup->getActionButtons()->getMapButton()->setChecked(false);
        displaySelectedGroup->getActionButtons()->getMapButton()->setToolTip("Click to display the selected point on the map");
    }
    /// enables the edit button
    displaySelectedGroup->getActionButtons()->getEditButton()->setEnabled(true);
    displaySelectedGroup->getActionButtons()->getEditButton()->setToolTip("Click to modify the selected point");
}

void LeftMenu::disableButtons(){
    displaySelectedGroup->getActionButtons()->getMapButton()->setCheckable(false);
    displaySelectedGroup->uncheck();
    /// resets the minus button
    displaySelectedGroup->getActionButtons()->getMinusButton()->setEnabled(false);
    displaySelectedGroup->getActionButtons()->getMinusButton()->setToolTip("Select a point and click here to remove it");
    /// resets the eye button
    displaySelectedGroup->getActionButtons()->getEyeButton()->setEnabled(false);
    displaySelectedGroup->getActionButtons()->getEyeButton()->setToolTip("Select a point and click here to access its information");
    /// resets the map button
    displaySelectedGroup->getActionButtons()->getMapButton()->setEnabled(false);
    displaySelectedGroup->getActionButtons()->getMapButton()->setToolTip("Select a point and click here to display or hide it on the map");
    /// resets the edit button
    displaySelectedGroup->getActionButtons()->getEditButton()->setEnabled(false);
    displaySelectedGroup->getActionButtons()->getEditButton()->setToolTip("Select a point and click here to modify it");
}

/// to factorize the remove point events
void LeftMenu::removePoint(){}
