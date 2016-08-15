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
#include "buttonmenu.h"
#include "colors.h"
#include "View/pathpainter.h"

LeftMenu::LeftMenu(MainWindow* _parent, std::shared_ptr<Points> const& _points,
                   const std::shared_ptr<Robots> &robots, const std::shared_ptr<Points> &pointViews,
                   const std::shared_ptr<Map> &_map, const std::shared_ptr<PathPainter> &pathPainter):
    QWidget(_parent), parent(_parent), points(_points), lastCheckedId("s")
{

    QVBoxLayout * leftLayout  = new QVBoxLayout();

    QVBoxLayout * globalLayout  = new QVBoxLayout(this);
    QHBoxLayout * topLayout  = new QHBoxLayout();

    returnButton = new QPushButton(QIcon(":/icons/arrowLeft.png"), " Return", this);
    returnButton->setAutoDefault(true);
    returnButton->setDefault(true);
    returnButton->setIconSize(parent->size()/10);
    returnButton->setStyleSheet("QPushButton{color: black;background-color:"+menu_button_color+"; border: none;}QPushButton:hover{background-color:"+button_hover_color+"; }");
    connect(returnButton, SIGNAL(clicked()), parent, SLOT(backEvent()));

    closeBtn = new QPushButton(QIcon(":/icons/cropped_close.png"), "", this);
    closeBtn->setIconSize(parent->size()/30);
    closeBtn->setFlat(true);
    closeBtn->setStyleSheet("QPushButton{color: black;background-color:"+menu_button_color+"; }QPushButton:hover{background-color:"+button_hover_color+"; }");

    returnButton->hide();
    topLayout->addWidget(returnButton,Qt::AlignLeft);

    topLayout->addWidget(closeBtn);

    leftLayout->addLayout(topLayout);
    connect(closeBtn, SIGNAL(clicked()), parent, SLOT(closeSlot()));

    /// to display the information relative to a point
    displaySelectedPoint = new DisplaySelectedPoint(parent, robots, _points, _map);
    leftLayout->addWidget(displaySelectedPoint);
    leftLayout->setAlignment(displaySelectedPoint, Qt::AlignLeft);

    /// to display the information relative to a group of points
    displaySelectedGroup = new DisplaySelectedGroup(parent, _points);
    leftLayout->addWidget(displaySelectedGroup);
    leftLayout->setAlignment(displaySelectedGroup, Qt::AlignLeft);

    /// The first menu with 3 buttons : Robots, Points, Map
    leftMenuWidget = new LeftMenuWidget(parent);
    leftLayout->addWidget(leftMenuWidget);

    /// Menu which display the list of points
    pointsLeftWidget = new PointsLeftWidget(parent, _points);
    leftLayout->addWidget(pointsLeftWidget);

    /// Menu which display the selected robot infos
    selectedRobotWidget = new SelectedRobotWidget(parent);

    connect(selectedRobotWidget, SIGNAL(showSelectedRobotWidget()), parent, SLOT(showHome()));
    connect(selectedRobotWidget, SIGNAL(hideSelectedRobotWidget()), parent, SLOT(hideHome()));
    leftLayout->addWidget(selectedRobotWidget);

    /// Menu which display the list of robots
    robotsLeftWidget = new RobotsLeftWidget(parent);
    robotsLeftWidget->setRobots(robots);
    leftLayout->addWidget(robotsLeftWidget,Qt::AlignTop);

    /// Menu which display the map menu in which user can save/load a map
    mapLeftWidget = new MapLeftWidget(parent);
    leftLayout->addWidget(mapLeftWidget);

    /// Menu to edit the selected robot
    editSelectedRobotWidget = new EditSelectedRobotWidget(parent, robots);
    leftLayout->addWidget(editSelectedRobotWidget);
    connect(editSelectedRobotWidget, SIGNAL(robotSaved()), parent, SLOT(robotSavedEvent()));
    connect(editSelectedRobotWidget, SIGNAL(showEditSelectedRobotWidget()), parent, SLOT(showHome()));
    connect(editSelectedRobotWidget, SIGNAL(hideEditSelectedRobotWidget()), parent, SLOT(hideHome()));

    /// Menu to edit the selected point
    createPointWidget = new CreatePointWidget(parent, pointViews);
    leftLayout->addWidget(createPointWidget);
    connect(createPointWidget, SIGNAL(pointSaved(QString, double, double, QString)), parent, SLOT(pointSavedEvent(QString, double, double, QString)));

    /// Menu which display the widget for the creation of a path
    pathCreationWidget = new PathCreationWidget(parent, _points);
    leftLayout->addWidget(pathCreationWidget);

    connect(pathCreationWidget, SIGNAL(addPathPoint(QString, double, double)), pathPainter.get(), SLOT(addPathPointSlot(QString, double, double)));
    connect(pathCreationWidget, SIGNAL(deletePathPoint(int)), pathPainter.get(), SLOT(deletePathPointSlot(int)));
    connect(pathCreationWidget, SIGNAL(orderPathPointChanged(int, int)), pathPainter.get(), SLOT(orderPathPointChangedSlot(int, int)));
    connect(pathCreationWidget, SIGNAL(resetPath()), pathPainter.get(), SLOT(resetPathSlot()));
    connect(pathCreationWidget, SIGNAL(setMessage(QString, QString)), parent, SLOT(setMessageTop(QString, QString)));
    connect(pathCreationWidget, SIGNAL(actionChanged(int, int, QString)), pathPainter.get(), SLOT(actionChangedSlot(int, int, QString)));
    connect(pathCreationWidget, SIGNAL(editPathPoint(int, QString, double, double)), pathPainter.get(), SLOT(editPathPointSlot(int, QString, double, double)));

   // connect(displaySelectedPoint->getDisplaySelectedPointRobots(), SIGNAL(setSelectedRobotFromPoint(QString)), parent, SLOT(setSelectedRobotFromPointSlot(QString)));
    connect(displaySelectedPoint->getActionButtons()->getMinusButton(), SIGNAL(clicked(bool)), parent, SLOT(removePointFromInformationMenu()));
    connect(displaySelectedPoint->getActionButtons()->getMapButton(), SIGNAL(clicked(bool)), parent, SLOT(displayPointMapEvent()));
    connect(displaySelectedPoint->getActionButtons()->getEditButton(), SIGNAL(clicked(bool)), parent, SLOT(editPointButtonEvent()));

    connect(displaySelectedGroup->getActionButtons()->getMinusButton(), SIGNAL(clicked(bool)), parent, SLOT(removePointFromGroupMenu()));
    connect(displaySelectedGroup->getActionButtons()->getEditButton(), SIGNAL(clicked(bool)), parent, SLOT(editPointFromGroupMenu()));
    connect(displaySelectedGroup->getActionButtons()->getGoButton(), SIGNAL(clicked(bool)), parent, SLOT(displayPointInfoFromGroupMenu()));
    connect(displaySelectedGroup->getActionButtons()->getMapButton(), SIGNAL(clicked(bool)), parent, SLOT(displayPointFromGroupMenu()));

    /// to check the name of a point being edited
    connect(displaySelectedPoint, SIGNAL(invalidName(QString,CreatePointWidget::Error)), parent, SLOT(setMessageCreationPoint(QString,CreatePointWidget::Error)));


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


    globalLayout->addLayout(leftLayout);
    globalLayout->setContentsMargins(0, 10, 0, 0);
    topLayout->setContentsMargins(0, 0, 0, 0);
    globalLayout->setSpacing(0);

    // set black background
    QPalette Pal(palette());
    Pal.setColor(QPalette::Background, left_menu_background_color);
    this->setAutoFillBackground(true);
    this->setPalette(Pal);

}

void LeftMenu::updateGroupDisplayed(std::shared_ptr<Points> const& _points, const QString groupIndex){
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

