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
#include "Controller/robotscontroller.h"

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
    leftLayout->addWidget(mainWindow->getRobotsController()->getRobotsLeftWidget());

    /// Menu which display the map menu in which user can save/load a map
    mapLeftWidget = new MapLeftWidget(this, mainWindow);
    mapLeftWidget->hide();
    leftLayout->addWidget(mapLeftWidget);

    /// Menu to edit the selected robot
    leftLayout->addWidget(mainWindow->getRobotsController()->getEditSelectedRobotWidget());

    /// Menu to edit the selected point
    leftLayout->addWidget(mainWindow->getPointsController()->getCreatePointWidget());

    /// Menu which display the informations of a path

    leftLayout->addWidget(mainWindow->getPathsController()->getDisplaySelectedPath());

    /// Menu which displays the groups of paths
    leftLayout->addWidget(mainWindow->getPathsController()->getGroupsPathsWidget());

    /// Menu which displays a particular group of paths
    leftLayout->addWidget(mainWindow->getPathsController()->getPathGroupDisplayed());

    leftLayout->addWidget(mainWindow->getPathsController()->getPathCreationWidget());
    hide();

    globalLayout->addLayout(leftLayout);

    /// globalLayout->addLayout reparents all the widgets contained in the left layout
    /// which is a behavior that we don't want because
    /// we sometimes cast the parent in mainwindow
    /// so we reparent everyone afterwards
    mainWindow->getPointsController()->getCreatePointWidget()->setParent(mainWindow);
    mainWindow->getRobotsController()->getEditSelectedRobotWidget()->setParent(mainWindow);
    mainWindow->getPathsController()->getDisplaySelectedPath()->setParent(mainWindow);
    mainWindow->getPathsController()->getGroupsPathsWidget()->setParent(mainWindow);
    mainWindow->getPathsController()->getPathGroupDisplayed()->setParent(mainWindow);
    mainWindow->getPathsController()->getPathCreationWidget()->setParent(mainWindow);

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
