#include "leftmenu.h"
#include "View/editselectedrobotwidget.h"
#include "View/createpointwidget.h"
#include "View/pointview.h"
#include "View/bottomlayout.h"
#include "View/leftmenuwidget.h"
#include "View/pointsleftwidget.h"
#include "View/robotsleftwidget.h"
#include "View/mapleftwidget.h"
#include "View/displayselectedpoint.h"
#include "View/displayselectedgroup.h"
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
#include "View/custompushbutton.h"
#include "View/displayselectedpointrobots.h"
#include "View/pathbuttongroup.h"
#include "Controller/pathscontroller.h"
#include "Controller/pointscontroller.h"

LeftMenu::LeftMenu(MainWindow* mainWindow, QSharedPointer<Points> const& points,
                   const QSharedPointer<Robots> &robots, const QSharedPointer<Map> &_map)
    : QWidget(mainWindow){

    QVBoxLayout* leftLayout  = new QVBoxLayout();

    QVBoxLayout* globalLayout  = new QVBoxLayout(this);
    QHBoxLayout* topLayout  = new QHBoxLayout();

    returnButton = new CustomPushButton(QIcon(":/icons/arrowLeft.png"), " Return", this);
    returnButton->setDefault(true);
    returnButton->setIconSize(xs_icon_size);
    connect(returnButton, SIGNAL(clicked()), mainWindow, SLOT(backEvent()));

    closeBtn = new CustomPushButton(QIcon(":/icons/close.png"), "", this);
    closeBtn->setIconSize(xxs_icon_size);
    closeBtn->addStyleSheet("QPushButton {padding-top: 15px; padding-bottom: 15px;}");

    returnButton->hide();
    topLayout->addWidget(returnButton, Qt::AlignLeft);

    topLayout->addWidget(closeBtn);

    globalLayout->addLayout(topLayout);
    connect(closeBtn, SIGNAL(clicked()), mainWindow, SLOT(closeSlot()));

    /// to display the information relative to a point
    leftLayout->addWidget(mainWindow->getPointsController()->getDisplaySelectedPoint());

    /// to display the information relative to a group of points
    leftLayout->addWidget(mainWindow->getPointsController()->getDisplaySelectedGroup());

    /// The first menu with 3 buttons : Robots, Points, Map
    leftMenuWidget = new LeftMenuWidget(this, points);

    connect(leftMenuWidget->getRobotBtn(), SIGNAL(clicked()), mainWindow, SLOT(robotBtnEvent()));
    connect(leftMenuWidget->getPointBtn(), SIGNAL(clicked()), mainWindow, SLOT(pointBtnEvent()));
    connect(leftMenuWidget->getMapBtn(), SIGNAL(clicked()), mainWindow, SLOT(mapBtnEvent()));
    connect(leftMenuWidget->getPathBtn(), SIGNAL(clicked()), mainWindow, SLOT(pathBtnEvent()));
    connect(leftMenuWidget, SIGNAL(resetPathPointViews()), mainWindow, SLOT(resetPathPointViewsSlot()));
    leftMenuWidget->hide();
    leftLayout->addWidget(leftMenuWidget);

    /// Menu which display the list of points
    leftLayout->addWidget(mainWindow->getPointsController()->getPointsLeftWidget());

    /// Menu which display the list of robots
    robotsLeftWidget = new RobotsLeftWidget(this, mainWindow, robots);
    robotsLeftWidget->hide();
    leftLayout->addWidget(robotsLeftWidget);

    /// Menu which display the map menu in which user can save/load a map
    mapLeftWidget = new MapLeftWidget(this, mainWindow);
    mapLeftWidget->hide();
    leftLayout->addWidget(mapLeftWidget);

    /// Menu to edit the selected robot
    editSelectedRobotWidget = new EditSelectedRobotWidget(this, mainWindow, points, robots, mainWindow->getPathsController()->getPaths());
    editSelectedRobotWidget->hide();
    leftLayout->addWidget(editSelectedRobotWidget);
    connect(editSelectedRobotWidget, SIGNAL(showEditSelectedRobotWidget()), mainWindow, SLOT(showEditHome()));
    connect(editSelectedRobotWidget, SIGNAL(hideEditSelectedRobotWidget()), mainWindow, SLOT(showAllHomes()));

    /// Menu to edit the selected point
    leftLayout->addWidget(mainWindow->getPointsController()->getCreatePointWidget());

    /// Menu which display the informations of a path

    leftLayout->addWidget(mainWindow->getPathsController()->getDisplaySelectedPath());

    /// Menu which displays the groups of paths
    leftLayout->addWidget(mainWindow->getPathsController()->getGroupsPathsWidget());

    /// Menu which displays a particular group of paths
    leftLayout->addWidget(mainWindow->getPathsController()->getPathGroupDisplayed());

    leftLayout->addWidget(mainWindow->getPathsController()->getpathCreationWidget());
    hide();

    globalLayout->addLayout(leftLayout);

    leftLayout->setAlignment(Qt::AlignTop);
    leftLayout->setAlignment(closeBtn, Qt::AlignTop | Qt::AlignRight);

    topLayout->setContentsMargins(10, 10, 10, 10);
    leftLayout->setContentsMargins(10, 10, 0, 10);
    globalLayout->setContentsMargins(0, 0, 0, 0);

    setMaximumWidth(mainWindow->width()/2);
    setMinimumWidth(mainWindow->width()/2);

    QPalette Pal(palette());
    Pal.setColor(QPalette::Background, left_menu_background_color);
    this->setAutoFillBackground(true);
    this->setPalette(Pal);
}

void LeftMenu::hideBackButton(void){
    if(returnButton != NULL)
        returnButton->hide();
}

void LeftMenu::showBackButton(const QString name){
    if(returnButton){
        returnButton->setText(name);
        returnButton->show();
    }
}

void LeftMenu::setEnableReturnCloseButtons(const bool enable){
    returnButton->setEnabled(enable);
    closeBtn->setEnabled(enable);
}
