#include "leftmenu.h"
#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QDebug>
#include <QScrollArea>
#include <QButtonGroup>
#include "Controller/mainwindow.h"
#include "Controller/Paths/pathscontroller.h"
#include "Controller/Points/pointscontroller.h"
#include "Controller/Robots/robotscontroller.h"
#include "Model/Points/points.h"
#include "Model/Other/xmlparser.h"
#include "View/Robots/editselectedrobotwidget.h"
#include "View/Points/createpointwidget.h"
#include "View/Points/pointview.h"
#include "View/BottomLayout/bottomlayout.h"
#include "View/LeftMenu/leftmenuwidget.h"
#include "View/Points/pointsleftwidget.h"
#include "View/Robots/robotsleftwidget.h"
#include "View/Map/mapleftwidget.h"
#include "View/Points/displayselectedpoint.h"
#include "View/Points/displayselectedgroup.h"
#include "View/Points/pointbuttongroup.h"
#include "View/Other/stylesettings.h"
#include "View/Paths/pathpainter.h"
#include "View/Other/custompushbutton.h"
#include "View/Points/displayselectedpointrobots.h"
#include "View/Paths/pathbuttongroup.h"
#include "View/Other/customscrollarea.h"

LeftMenu::LeftMenu(MainWindow* mainWindow)
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
    leftMenuWidget = new LeftMenuWidget(this);

    connect(leftMenuWidget->getRobotBtn(), SIGNAL(clicked()), mainWindow, SLOT(robotBtnEvent()));
    connect(leftMenuWidget->getPointBtn(), SIGNAL(clicked()), mainWindow, SLOT(pointBtnEvent()));
    connect(leftMenuWidget->getMapBtn(), SIGNAL(clicked()), mainWindow, SLOT(mapBtnEvent()));
    connect(leftMenuWidget->getPathBtn(), SIGNAL(clicked()), mainWindow, SLOT(pathBtnEvent()));
    connect(leftMenuWidget, SIGNAL(resetPathPointViews()), mainWindow, SLOT(resetPathPointViewsSlot()));
    connect(leftMenuWidget, SIGNAL(resetPointViews()), mainWindow->getPointsController(), SLOT(resetPointViewsSlot()));
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
