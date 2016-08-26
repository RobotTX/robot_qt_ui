#include "leftmenu.h"
#include "View/editselectedrobotwidget.h"
#include "View/createpointwidget.h"
#include "View/pointview.h"
#include "View/bottomlayout.h"
#include "View/leftmenuwidget.h"
#include "View/pointsleftwidget.h"
#include "View/selectedrobotwidget.h"
#include "View/robotsleftwidget.h"
#include "View/mapleftwidget.h"
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
#include "customscrollarea.h"
#include <QButtonGroup>
#include "Model/points.h"
#include "Model/xmlparser.h"
#include "Controller/mainwindow.h"
#include "stylesettings.h"
#include "View/pathpainter.h"
#include "View/displayselectedpath.h"
#include "View/groupspathswidget.h"
#include "Model/paths.h"
#include "View/displaypathgroup.h"
#include "View/custompushbutton.h"
#include "View/displayselectedpointrobots.h"

LeftMenu::LeftMenu(MainWindow* _mainWindow, QSharedPointer<Points> const& _points, QSharedPointer<Paths> const& _paths,
                   const QSharedPointer<Robots> &robots, const QSharedPointer<Points> &pointViews,
                   const QSharedPointer<Map> &_map, const PathPainter *pathPainter):
    QWidget(_mainWindow), mainWindow(_mainWindow), points(_points), paths(_paths), lastCheckedId("s")
{

    QVBoxLayout * leftLayout  = new QVBoxLayout();

    QVBoxLayout * globalLayout  = new QVBoxLayout(this);
    QHBoxLayout * topLayout  = new QHBoxLayout();

    returnButton = new CustomPushButton(QIcon(":/icons/arrowLeft.png"), " Return", this);
    //returnButton->setAutoDefault(true);
    returnButton->setDefault(true);
    returnButton->setIconSize(normal_icon_size);
    connect(returnButton, SIGNAL(clicked()), mainWindow, SLOT(backEvent()));

    closeBtn = new CustomPushButton(QIcon(":/icons/cropped_close.png"), "", this);
    closeBtn->setIconSize(small_icon_size);
    closeBtn->addStyleSheet("QPushButton {padding-top: 15px; padding-bottom: 15px;}");

    returnButton->hide();
    topLayout->addWidget(returnButton,Qt::AlignLeft);

    topLayout->addWidget(closeBtn);

    globalLayout->addLayout(topLayout);
    connect(closeBtn, SIGNAL(clicked()), mainWindow, SLOT(closeSlot()));

    /// to display the information relative to a point
    displaySelectedPoint = new DisplaySelectedPoint(this, robots, _points, _map);
    //displaySelectedPoint->setMaximumWidth(mainWindow->width()*4/10);
    connect(displaySelectedPoint->getDisplaySelectedPointRobots(), SIGNAL(setSelectedRobotFromPoint(QString)), mainWindow, SLOT(setSelectedRobotFromPointSlot(QString)));
    displaySelectedPoint->hide();
    leftLayout->addWidget(displaySelectedPoint);
    leftLayout->setAlignment(displaySelectedPoint, Qt::AlignLeft);

    /// to display the information relative to a group of points
    displaySelectedGroup = new DisplaySelectedGroup(this, _points);
    //displaySelectedGroup->setMaximumWidth(mainWindow->width()*4/10);
    connect(displaySelectedGroup, SIGNAL(resetPathPointViews()), mainWindow, SLOT(resetPathPointViewsSlot()));
    displaySelectedGroup->hide();
    leftLayout->addWidget(displaySelectedGroup);
    leftLayout->setAlignment(displaySelectedGroup, Qt::AlignLeft);

    /// The first menu with 3 buttons : Robots, Points, Map
    leftMenuWidget = new LeftMenuWidget(this, points);
    //leftMenuWidget->setMaximumWidth(mainWindow->width()*4/10);

    connect(leftMenuWidget->getRobotBtn(), SIGNAL(clicked()), mainWindow, SLOT(robotBtnEvent()));
    connect(leftMenuWidget->getPointBtn(), SIGNAL(clicked()), mainWindow, SLOT(pointBtnEvent()));
    connect(leftMenuWidget->getMapBtn(), SIGNAL(clicked()), mainWindow, SLOT(mapBtnEvent()));
    connect(leftMenuWidget->getPathBtn(), SIGNAL(clicked()), mainWindow, SLOT(pathBtnEvent()));
    connect(leftMenuWidget, SIGNAL(resetPathPointViews()), mainWindow, SLOT(resetPathPointViewsSlot()));
    leftMenuWidget->hide();
    leftLayout->addWidget(leftMenuWidget);

    /// Menu which display the list of points
    pointsLeftWidget = new PointsLeftWidget(this, mainWindow, _points);
    //pointsLeftWidget->setMaximumWidth(mainWindow->width()*4/10);
    pointsLeftWidget->hide();
    leftLayout->addWidget(pointsLeftWidget);

    /// Menu which display the selected robot infos
    selectedRobotWidget = new SelectedRobotWidget(this, mainWindow);
    //selectedRobotWidget->setMaximumWidth(mainWindow->width()*4/10);

    connect(selectedRobotWidget, SIGNAL(showSelectedRobotWidget()), mainWindow, SLOT(showHome()));
    connect(selectedRobotWidget, SIGNAL(hideSelectedRobotWidget()), mainWindow, SLOT(hideHome()));
    selectedRobotWidget->hide();
    leftLayout->addWidget(selectedRobotWidget);

    /// Menu which display the list of robots
    robotsLeftWidget = new RobotsLeftWidget(this, mainWindow, robots);
    //robotsLeftWidget->setMaximumWidth(mainWindow->width()*4/10);
    robotsLeftWidget->hide();
    leftLayout->addWidget(robotsLeftWidget);

    /// Menu which display the map menu in which user can save/load a map
    mapLeftWidget = new MapLeftWidget(this, mainWindow);
    //mapLeftWidget->setMaximumWidth(mainWindow->width()*4/10);
    mapLeftWidget->hide();
    leftLayout->addWidget(mapLeftWidget);

    /// Menu to edit the selected robot
    editSelectedRobotWidget = new EditSelectedRobotWidget(this, mainWindow, robots);
    //editSelectedRobotWidget->setMaximumWidth(mainWindow->width()*4/10);
    editSelectedRobotWidget->hide();
    leftLayout->addWidget(editSelectedRobotWidget);
    connect(editSelectedRobotWidget, SIGNAL(robotSaved()), mainWindow, SLOT(robotSavedEvent()));
    connect(editSelectedRobotWidget, SIGNAL(showEditSelectedRobotWidget()), mainWindow, SLOT(showEditHome()));
    connect(editSelectedRobotWidget, SIGNAL(hideEditSelectedRobotWidget()), mainWindow, SLOT(hideHome()));

    /// Menu to edit the selected point
    createPointWidget = new CreatePointWidget(this, mainWindow, pointViews);
    //createPointWidget->setMaximumWidth(mainWindow->width()*4/10);
    createPointWidget->hide();
    leftLayout->addWidget(createPointWidget);
    connect(createPointWidget, SIGNAL(pointSaved(QString, double, double, QString)), mainWindow, SLOT(pointSavedEvent(QString, double, double, QString)));

    /// Menu which displays the widget for the creation of a path
    pathCreationWidget = new PathCreationWidget(this, mainWindow, _points);
    //pathCreationWidget->setMaximumWidth(mainWindow->width()*4/10);
    pathCreationWidget->hide();
    leftLayout->addWidget(pathCreationWidget);

    /// Menu which display the informations of a path
    displaySelectedPath = new DisplaySelectedPath(this, mainWindow);
    //displaySelectedPath->setMaximumWidth(mainWindow->width()*4/10);
    displaySelectedPath->hide();
    leftLayout->addWidget(displaySelectedPath);

    connect(pathCreationWidget, SIGNAL(addPathPoint(QString, double, double)), pathPainter, SLOT(addPathPointSlot(QString, double, double)));
    connect(pathCreationWidget, SIGNAL(deletePathPoint(int)), pathPainter, SLOT(deletePathPointSlot(int)));
    connect(pathCreationWidget, SIGNAL(orderPathPointChanged(int, int)), pathPainter, SLOT(orderPathPointChangedSlot(int, int)));
    connect(pathCreationWidget, SIGNAL(resetPath()), pathPainter, SLOT(resetPathSlot()));
    connect(pathCreationWidget, SIGNAL(setMessage(QString, QString)), mainWindow, SLOT(setMessageTop(QString, QString)));
    connect(pathCreationWidget, SIGNAL(actionChanged(int, int, QString)), pathPainter, SLOT(actionChangedSlot(int, int, QString)));
    connect(pathCreationWidget, SIGNAL(editPathPoint(int, QString, double, double)), pathPainter, SLOT(editPathPointSlot(int, QString, double, double)));

    connect(displaySelectedPoint->getActionButtons()->getMinusButton(), SIGNAL(clicked(bool)), mainWindow, SLOT(removePointFromInformationMenu()));
    connect(displaySelectedPoint->getActionButtons()->getMapButton(), SIGNAL(clicked(bool)), mainWindow, SLOT(displayPointMapEvent()));
    connect(displaySelectedPoint->getActionButtons()->getEditButton(), SIGNAL(clicked(bool)), mainWindow, SLOT(editPointButtonEvent()));

    connect(displaySelectedGroup->getActionButtons()->getMinusButton(), SIGNAL(clicked(bool)), mainWindow, SLOT(removePointFromGroupMenu()));
    connect(displaySelectedGroup->getActionButtons()->getEditButton(), SIGNAL(clicked(bool)), mainWindow, SLOT(editPointFromGroupMenu()));
    connect(displaySelectedGroup->getActionButtons()->getGoButton(), SIGNAL(clicked(bool)), mainWindow, SLOT(displayPointInfoFromGroupMenu()));
    connect(displaySelectedGroup->getActionButtons()->getMapButton(), SIGNAL(clicked(bool)), mainWindow, SLOT(displayPointFromGroupMenu()));

    /// to check the name of a point being edited
    connect(displaySelectedPoint, SIGNAL(invalidName(QString,CreatePointWidget::Error)), mainWindow, SLOT(setMessageCreationPoint(QString,CreatePointWidget::Error)));



    /// Menu which displays the groups of paths
    groupsPathsWidget = new GroupsPathsWidget(this, _mainWindow, paths);
    //groupsPathsWidget->setMaximumWidth(mainWindow->width()*4/10);
    groupsPathsWidget->hide();
    leftLayout->addWidget(groupsPathsWidget);

    connect(groupsPathsWidget->getActionButtons()->getGoButton(), SIGNAL(clicked()), mainWindow, SLOT(displayGroupPaths()));
    connect(groupsPathsWidget->getActionButtons()->getEditButton(), SIGNAL(clicked()), mainWindow, SLOT(editGroupPaths()));
    connect(groupsPathsWidget->getActionButtons()->getPlusButton(), SIGNAL(clicked()), mainWindow, SLOT(createGroupPaths()));
    connect(groupsPathsWidget->getActionButtons()->getMinusButton(), SIGNAL(clicked()), mainWindow, SLOT(deleteGroupPaths()));

    /// Menu which displays a particular group of paths
    pathGroup = new DisplayPathGroup(this, paths);
    //pathGroup->setMaximumWidth(mainWindow->width()*4/10);
    pathGroup->hide();
    leftLayout->addWidget(pathGroup);

    connect(pathGroup->getActionButtons()->getGoButton(), SIGNAL(clicked()), mainWindow, SLOT(displayPath()));
    connect(pathGroup->getActionButtons()->getPlusButton(), SIGNAL(clicked()), mainWindow, SLOT(createPath()));
    connect(pathGroup->getActionButtons()->getMinusButton(), SIGNAL(clicked()), mainWindow, SLOT(deletePath()));
    connect(pathGroup->getActionButtons()->getMapButton(), SIGNAL(toggled(bool)), mainWindow, SLOT(displayPathOnMap(bool)));
    connect(pathGroup->getActionButtons()->getEditButton(), SIGNAL(clicked()), mainWindow, SLOT(editPath()));

    hide();

    globalLayout->addLayout(leftLayout);
    /*globalLayout->setContentsMargins(0, 10, 0, 0);
    topLayout->setContentsMargins(0, 0, 0, 0);
    globalLayout->setSpacing(0);*/

    // set black background

    leftLayout->setAlignment(Qt::AlignTop);
    leftLayout->setAlignment(closeBtn, Qt::AlignTop | Qt::AlignRight);
/*
   for (int i = 0; i < leftLayout->count(); ++i) {
        QWidget *widget = leftLayout->itemAt(i)->widget();
        if (widget != NULL) {
            widget->setMinimumWidth(1);
        }
    }*/

    setMaximumWidth(_mainWindow->width()/2);
    setMinimumWidth(_mainWindow->width()/2);

    QPalette Pal(palette());
    Pal.setColor(QPalette::Background, left_menu_background_color);
    this->setAutoFillBackground(true);
    this->setPalette(Pal);
}

void LeftMenu::updateGroupDisplayed(const QString groupIndex){
    displaySelectedGroup->getPointButtonGroup()->setGroup(groupIndex);
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

void LeftMenu::setEnableReturnCloseButtons(bool enable){
    returnButton->setEnabled(enable);
    closeBtn->setEnabled(enable);
}
