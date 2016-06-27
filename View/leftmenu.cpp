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
LeftMenu::LeftMenu(QMainWindow* parent, Points const& points, const std::shared_ptr<Robots> &robots, PointsView * const &pointViews):QWidget(parent){
   // leftLayout = new QVBoxLayout(this);

    VerticalScrollArea* scroll = new VerticalScrollArea(parent);
    QVBoxLayout * leftLayout  = new QVBoxLayout();
    QWidget * inWidget  = new QWidget();
    QVBoxLayout * globalLayout  = new QVBoxLayout(this);

    QPushButton* closeBtn = new QPushButton(QIcon(":/icons/cropped_close.png"), "", this);
    closeBtn->setIconSize(parent->size()/30);
    closeBtn->setFlat(true);
    //closeBtn->setStyleSheet("QPushButton { padding: 5px;}");
    //closeBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    leftLayout->addWidget(closeBtn);
    connect(closeBtn, SIGNAL(clicked()), parent, SLOT(closeSlot()));

    /// to display the information relative to a point
    displaySelectedPoint = new DisplaySelectedPoint(parent, points);

    leftLayout->addWidget(displaySelectedPoint);
    /// to display the information relative to a group of points
    displaySelectedGroup = new DisplaySelectedGroup(parent, points);
    leftLayout->addWidget(displaySelectedGroup);

    /// The first menu with 3 buttons : Robots, Points, Map
    leftMenuWidget = new LeftMenuWidget(parent);
    leftLayout->addWidget(leftMenuWidget);

    /// Menu which display the list of points
    pointsLeftWidget = new PointsLeftWidget(parent, points);
    leftLayout->addWidget(pointsLeftWidget);

    /// Menu which display the selected robot infos
    selectedRobotWidget = new SelectedRobotWidget(parent);
    connect(selectedRobotWidget, SIGNAL(selectHome(RobotView*)), parent, SLOT(selectHomeEvent()));
    connect(selectedRobotWidget, SIGNAL(showSelectedRobotWidget()), parent, SLOT(showHome()));
    connect(selectedRobotWidget, SIGNAL(hideSelectedRobotWidget()), parent, SLOT(hideHome()));
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
    connect(editSelectedRobotWidget, SIGNAL(showEditSelectedRobotWidget()), parent, SLOT(showHome()));
    connect(editSelectedRobotWidget, SIGNAL(hideEditSelectedRobotWidget()), parent, SLOT(hideHome()));

    /// Menu which display the selected point infos
    selectedPointWidget = new SelectedPointWidget(parent);
    leftLayout->addWidget(selectedPointWidget);

    /// Menu to edit the selected point
    editSelectedPointWidget = new EditSelectedPointWidget(parent, pointViews);
    leftLayout->addWidget(editSelectedPointWidget);
    connect(editSelectedPointWidget, SIGNAL(pointSaved(int, double, double, QString)), parent, SLOT(pointSavedEvent(int, double, double, QString)));

    /// Menu which display the widget for the creation of a path
    pathCreationWidget = new PathCreationWidget(parent, points);
    leftLayout->addWidget(pathCreationWidget);
    connect(pathCreationWidget, SIGNAL(updatePathPointToPainter(QVector<Point>*)), parent, SLOT(updatePathPointToPainter(QVector<Point>*)));
    connect(pathCreationWidget, SIGNAL(hidePathCreationWidget()), parent, SLOT(hidePathCreationWidget()));
    connect(pathCreationWidget, SIGNAL(editTmpPathPoint(int, Point*, int)), parent, SLOT(editTmpPathPointSlot(int, Point*, int)));
    connect(pathCreationWidget, SIGNAL(saveEditPathPoint()), parent, SLOT(saveTmpEditPathPointSlot()));
    connect(pathCreationWidget, SIGNAL(setMessage(QString, QString)), parent, SLOT(setMessageTop(QString, QString)));


    connect(displaySelectedPoint->getBackButton(), SIGNAL(clicked(bool)), parent, SLOT(pointBtnEvent()));
    /// Last widget visited, used to know where to go back when pressing the return button
    lastWidget = NULL;

    connect(displaySelectedPoint->getMinusButton(), SIGNAL(clicked(bool)), parent, SLOT(removePointFromInformationMenu()));
    connect(displaySelectedPoint->getMapButton(), SIGNAL(clicked(bool)), parent, SLOT(displayPointMapEvent()));
    connect(displaySelectedPoint->getEditButton(), SIGNAL(clicked(bool)), parent, SLOT(editPointButtonEvent(bool)));

    connect(displaySelectedGroup->getBackButton(), SIGNAL(clicked(bool)), parent, SLOT(pointBtnEvent()));
    connect(displaySelectedGroup->getMinusButton(), SIGNAL(clicked(bool)), parent, SLOT(removePointFromGroupMenu()));
    connect(displaySelectedGroup->getEditButton(), SIGNAL(clicked(bool)), parent, SLOT(editPointFromGroupMenu()));
    connect(displaySelectedGroup->getEyeButton(), SIGNAL(clicked(bool)), parent, SLOT(displayPointInfoFromGroupMenu()));
    connect(displaySelectedGroup->getMapButton(), SIGNAL(clicked(bool)), parent, SLOT(displayPointFromGroupMenu()));

    hide();
    leftLayout->setContentsMargins(0,0,0,0);
    setMaximumWidth(parent->width()*4/10);
    setMinimumWidth(parent->width()*4/10);
    leftLayout->setAlignment(Qt::AlignTop);
    leftLayout->setAlignment(closeBtn, Qt::AlignTop | Qt::AlignRight);

    for (int i = 0; i < leftLayout->count(); ++i) {
        QWidget *widget = leftLayout->itemAt(i)->widget();
        if (widget != NULL) {
            widget->setMinimumWidth(1);
        }
    }

    inWidget->setLayout(leftLayout);
    scroll->setWidget(inWidget);
    globalLayout->addWidget(scroll);
    globalLayout->setContentsMargins(0, 0, 0, 0);
    this->setLayout(globalLayout);
}

void LeftMenu::updateGroupDisplayed(const Points& _points, const int groupIndex){
    displaySelectedGroup->getPointButtonGroup()->setGroup(_points, groupIndex);
}
