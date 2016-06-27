#include "leftmenu.h"
#include "View/editselectedrobotwidget.h"
#include "View/selectedpointwidget.h"
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
#include "View/groupeditwindow.h"
#include <QMainWindow>
#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QDebug>
#include "View/pointbuttongroup.h"
#include <QScrollArea>
#include "verticalscrollarea.h"
#include <QButtonGroup>

LeftMenu::LeftMenu(QMainWindow* _parent, Points const& points, const std::shared_ptr<Robots> &robots, PointsView * const &pointViews): QWidget(_parent), parent(_parent){
   // leftLayout = new QVBoxLayout(this);

    QScrollArea * scroll = new VerticalScrollArea(_parent);
    QVBoxLayout * leftLayout  = new QVBoxLayout();
    QWidget * inWidget  = new QWidget();
    QVBoxLayout * globalLayout  = new QVBoxLayout(this);

    QPushButton* closeBtn = new QPushButton(QIcon(":/icons/cropped_close.png"), "", this);
    closeBtn->setIconSize(_parent->size()/30);
    closeBtn->setFlat(true);
    //closeBtn->setStyleSheet("QPushButton { padding: 5px;}");
    //closeBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    leftLayout->addWidget(closeBtn);
    connect(closeBtn, SIGNAL(clicked()), _parent, SLOT(closeSlot()));

    /// to display the information relative to a point
    displaySelectedPoint = new DisplaySelectedPoint(_parent, points);
    leftLayout->addWidget(displaySelectedPoint);
    leftLayout->setAlignment(displaySelectedPoint, Qt::AlignLeft);

    /// to display the information relative to a group of points
    displaySelectedGroup = new DisplaySelectedGroup(_parent, points);
    leftLayout->addWidget(displaySelectedGroup);
    leftLayout->setAlignment(displaySelectedGroup, Qt::AlignLeft);

    /// The first menu with 3 buttons : Robots, Points, Map
    leftMenuWidget = new LeftMenuWidget(_parent);
    leftLayout->addWidget(leftMenuWidget);

    /// Menu which display the list of points
    pointsLeftWidget = new PointsLeftWidget(_parent, points);
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

    /// Menu which display the selected point infos
    selectedPointWidget = new SelectedPointWidget(_parent);
    leftLayout->addWidget(selectedPointWidget);

    /// Menu to edit the selected point
    editSelectedPointWidget = new EditSelectedPointWidget(_parent, pointViews);
    leftLayout->addWidget(editSelectedPointWidget);
    connect(editSelectedPointWidget, SIGNAL(pointSaved(int, double, double, QString)), _parent, SLOT(pointSavedEvent(int, double, double, QString)));

    /// Menu which display the widget for the creation of a path
    pathCreationWidget = new PathCreationWidget(_parent, points);
    leftLayout->addWidget(pathCreationWidget);
    connect(pathCreationWidget, SIGNAL(updatePathPointToPainter(QVector<Point>*)), _parent, SLOT(updatePathPointToPainter(QVector<Point>*)));
    connect(pathCreationWidget, SIGNAL(hidePathCreationWidget()), _parent, SLOT(hidePathCreationWidget()));
    connect(pathCreationWidget, SIGNAL(editTmpPathPoint(int, Point*, int)), _parent, SLOT(editTmpPathPointSlot(int, Point*, int)));
    connect(pathCreationWidget, SIGNAL(saveEditPathPoint()), _parent, SLOT(saveTmpEditPathPointSlot()));
    connect(pathCreationWidget, SIGNAL(setMessage(QString, QString)), _parent, SLOT(setMessageTop(QString, QString)));


    connect(displaySelectedPoint->getBackButton(), SIGNAL(clicked(bool)), _parent, SLOT(pointBtnEvent()));
    /// Last widget visited, used to know where to go back when pressing the return button
    lastWidget = NULL;

    connect(displaySelectedPoint->getMinusButton(), SIGNAL(clicked(bool)), _parent, SLOT(removePointFromInformationMenu()));
    connect(displaySelectedPoint->getMapButton(), SIGNAL(clicked(bool)), _parent, SLOT(displayPointMapEvent()));
    connect(displaySelectedPoint->getEditButton(), SIGNAL(clicked(bool)), _parent, SLOT(editPointButtonEvent(bool)));

    connect(displaySelectedGroup->getBackButton(), SIGNAL(clicked(bool)), _parent, SLOT(pointBtnEvent()));
    connect(displaySelectedGroup->getMinusButton(), SIGNAL(clicked(bool)), _parent, SLOT(removePointFromGroupMenu()));
    connect(displaySelectedGroup->getEditButton(), SIGNAL(clicked(bool)), _parent, SLOT(editPointFromGroupMenu()));
    connect(displaySelectedGroup->getEyeButton(), SIGNAL(clicked(bool)), _parent, SLOT(displayPointInfoFromGroupMenu()));
    connect(displaySelectedGroup->getMapButton(), SIGNAL(clicked(bool)), _parent, SLOT(displayPointFromGroupMenu()));

    /// to handle double clicks in the groups menu
    foreach(QAbstractButton* button, displaySelectedGroup->getPointButtonGroup()->getButtonGroup()->buttons())
        connect(button, SIGNAL(doubleClick(int)), _parent, SLOT(doubleClickOnPoint(int)));

    hide();
    leftLayout->setContentsMargins(0,0,0,0);
    setMaximumWidth(_parent->width()*4/10);
    setMinimumWidth(_parent->width()*4/10);
    leftLayout->setAlignment(Qt::AlignTop);
    leftLayout->setAlignment(closeBtn, Qt::AlignTop | Qt::AlignRight);

    inWidget->setLayout(leftLayout);
    scroll->setWidget(inWidget);
    globalLayout->addWidget(scroll);
    this->setLayout(globalLayout);
}

void LeftMenu::updateGroupDisplayed(const Points& _points, const int groupIndex){
    displaySelectedGroup->getPointButtonGroup()->setGroup(_points, groupIndex);
    /// to handle double clicks in the groups menu
    foreach(QAbstractButton* button, displaySelectedGroup->getPointButtonGroup()->getButtonGroup()->buttons())
        connect(button, SIGNAL(doubleClick(int)), parent, SLOT(doubleClickOnPoint(int)));
}
