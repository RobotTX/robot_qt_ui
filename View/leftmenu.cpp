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

LeftMenu::LeftMenu(MainWindow* mainWindow, QSharedPointer<Points> const& _points,
                   const QSharedPointer<Robots> &robots, const QSharedPointer<Map> &_map)
    : QWidget(mainWindow), points(_points){

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

    // the 4 next lines of codes are causing a warning about the layout
    /// to display the information relative to a point
    displaySelectedPoint = new DisplaySelectedPoint(this, robots, _points, _map);
    connect(displaySelectedPoint->getDisplaySelectedPointRobots(), SIGNAL(setSelectedRobotFromPoint(QString)), mainWindow, SLOT(setSelectedRobotFromPointSlot(QString)));
    displaySelectedPoint->hide();
    leftLayout->addWidget(displaySelectedPoint);

    /// to display the information relative to a group of points
    displaySelectedGroup = new DisplaySelectedGroup(this, _points);
    connect(displaySelectedGroup, SIGNAL(resetPathPointViews()), mainWindow, SLOT(resetPathPointViewsSlot()));
    displaySelectedGroup->hide();
    leftLayout->addWidget(displaySelectedGroup);

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
    pointsLeftWidget = new PointsLeftWidget(this, mainWindow, _points);
    pointsLeftWidget->hide();
    leftLayout->addWidget(pointsLeftWidget);

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
    createPointWidget = new CreatePointWidget(this, mainWindow, points);
    createPointWidget->hide();
    leftLayout->addWidget(createPointWidget);
    connect(createPointWidget, SIGNAL(pointSaved(QString, double, double, QString)), mainWindow, SLOT(pointSavedEvent(QString, double, double, QString)));

    /// Menu which display the informations of a path

    leftLayout->addWidget(mainWindow->getPathsController()->getDisplaySelectedPath());

    connect(displaySelectedPoint->getActionButtons()->getMinusButton(), SIGNAL(clicked(bool)), mainWindow, SLOT(removePointFromInformationMenu()));
    connect(displaySelectedPoint->getActionButtons()->getMapButton(), SIGNAL(clicked(bool)), mainWindow, SLOT(displayPointMapEvent()));
    connect(displaySelectedPoint->getActionButtons()->getEditButton(), SIGNAL(clicked(bool)), mainWindow, SLOT(editPointButtonEvent()));
    /// to remove the point by pressing the delete key
    connect(displaySelectedPoint, SIGNAL(removePoint()), mainWindow, SLOT(removePointFromInformationMenu()));

    /// to cancel edition of a point on hide event
    connect(displaySelectedPoint, SIGNAL(cancelEditionPoint()), mainWindow, SLOT(cancelEvent()));

    connect(displaySelectedGroup->getActionButtons()->getMinusButton(), SIGNAL(clicked(bool)), mainWindow, SLOT(removePointFromGroupMenu()));
    connect(displaySelectedGroup->getActionButtons()->getEditButton(), SIGNAL(clicked(bool)), mainWindow, SLOT(editPointFromGroupMenu()));
    connect(displaySelectedGroup->getActionButtons()->getGoButton(), SIGNAL(clicked(bool)), mainWindow, SLOT(displayPointInfoFromGroupMenu()));
    connect(displaySelectedGroup->getActionButtons()->getMapButton(), SIGNAL(clicked(bool)), mainWindow, SLOT(displayPointFromGroupMenu()));
    /// to remove the point by pressing the delete key
    connect(displaySelectedGroup, SIGNAL(removePoint()), mainWindow, SLOT(removePointFromGroupMenu()));

    /// to check the name of a point being edited
    connect(displaySelectedPoint, SIGNAL(invalidName(QString, CreatePointWidget::Error)), mainWindow, SLOT(setMessageCreationPoint(QString,CreatePointWidget::Error)));

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

void LeftMenu::updateGroupDisplayed(const QString groupName){
    displaySelectedGroup->getPointButtonGroup()->setGroup(groupName);
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
