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

LeftMenu::LeftMenu(QMainWindow* parent, Points const& points, Robots * const &robots, PointsView * const &pointViews){
    leftLayout = new QVBoxLayout();

    /// to display the information relative to a point

    displaySelectedPoint = new DisplaySelectedPoint(parent, points);

    leftLayout->addWidget(displaySelectedPoint);
qDebug() << "ok";
    /// to display the information relative to a group of points
    displaySelectedGroup = new DisplaySelectedGroup(parent, points);
    qDebug() << "ok";
    leftLayout->addWidget(displaySelectedGroup);

    qDebug() << "ok";

    /// The first menu with 3 buttons : Robots, Points, Map
    leftMenuWidget = new LeftMenuWidget(parent);
    leftLayout->addWidget(leftMenuWidget);

    /// Menu which display the list of points
    pointsLeftWidget = new PointsLeftWidget(parent, points);
    leftLayout->addWidget(pointsLeftWidget);

    /// Menu which display the selected robot infos
    selectedRobotWidget = new SelectedRobotWidget(parent);
    connect(selectedRobotWidget, SIGNAL(selectHome(RobotView*)), parent, SLOT(selectHomeEvent()));
    connect(selectedRobotWidget, SIGNAL(showHome(RobotView*)), parent, SLOT(showHomeEvent()));
    leftLayout->addWidget(selectedRobotWidget);

    /// Menu which display the list of robots
    robotsLeftWidget = new RobotsLeftWidget(parent);
    robotsLeftWidget->setRobots(robots);
    leftLayout->addWidget(robotsLeftWidget);

    /// Menu which display the map menu in which user can save/load a map
    mapLeftWidget = new MapLeftWidget(parent);
    leftLayout->addWidget(mapLeftWidget);

    /// Menu to edit the selected robot
    editSelectedRobotWidget = new EditSelectedRobotWidget(parent, robots);
    leftLayout->addWidget(editSelectedRobotWidget);
    connect(editSelectedRobotWidget, SIGNAL(robotSaved()), parent, SLOT(robotSavedEvent()));

    /// Menu which display the selected point infos
    selectedPointWidget = new SelectedPointWidget(parent);
    leftLayout->addWidget(selectedPointWidget);

    /// Menu to edit the selected point
    editSelectedPointWidget = new EditSelectedPointWidget(parent, pointViews);
    leftLayout->addWidget(editSelectedPointWidget);
    connect(editSelectedPointWidget, SIGNAL(pointSaved()), parent, SLOT(pointSavedEvent()));

    /// Menu which display the widget for the creation of a path
    pathCreationWidget = new PathCreationWidget(parent, points);
    leftLayout->addWidget(pathCreationWidget);
    connect(pathCreationWidget, SIGNAL(updatePathPointToPainter(QVector<Point>*)), parent, SLOT(updatePathPointToPainter(QVector<Point>*)));
    connect(pathCreationWidget, SIGNAL(hidePathCreationWidget()), parent, SLOT(hidePathCreationWidget()));
    connect(pathCreationWidget, SIGNAL(editTmpPathPoint(int, Point*, int)), parent, SLOT(editTmpPathPointSlot(int, Point*, int)));
    connect(pathCreationWidget, SIGNAL(saveEditPathPoint()), parent, SLOT(saveTmpEditPathPointSlot()));


    connect(displaySelectedPoint->getBackButton(), SIGNAL(clicked()), parent, SLOT(pointBtnEvent()));
    /// Last widget visited, used to know where to go back when pressing the return button
    lastWidget = NULL;


    connect(displaySelectedPoint->getMinusButton(), SIGNAL(clicked(bool)), parent, SLOT(removePointFromInformationMenu()));
    connect(displaySelectedPoint->getMapButton(), SIGNAL(clicked(bool)), parent, SLOT(displayPointMapEvent()));
    connect(displaySelectedPoint->getEditButton(), SIGNAL(clicked(bool)), parent, SLOT(editPointButtonEvent()));


    connect(displaySelectedGroup->getBackButton(), SIGNAL(clicked(bool)), parent, SLOT(pointBtnEvent()));
    // prob need a different event
    //connect(displaySelectedGroup->getMapButton(), SIGNAL(clicked(bool)), parent, SLOT(displayGroupMapEvent()));

    hide();
    leftLayout->setContentsMargins(0,0,0,0);
    setMaximumWidth(parent->width()*4/10);
    setMinimumWidth(parent->width()*4/10);
    leftLayout->setAlignment(Qt::AlignTop);

    setLayout(leftLayout);
}


LeftMenu::~LeftMenu(){
    delete leftLayout;
    delete lastWidget;
    delete leftMenuWidget;
    delete pointsLeftWidget;
    delete selectedRobotWidget;
    delete robotsLeftWidget;
    delete mapLeftWidget;
    delete editSelectedRobotWidget;
    delete selectedPointWidget;
    delete editSelectedPointWidget;
    delete displaySelectedPoint;
    delete displaySelectedGroup;
    delete pathCreationWidget;
}

void LeftMenu::updateGroupDisplayed(const Points& _points, const int groupIndex){
    displaySelectedGroup->getPointButtonGroup()->setGroup(_points, groupIndex);
}


