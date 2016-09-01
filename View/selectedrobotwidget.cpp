#include "selectedrobotwidget.h"
#include "View/robotview.h"
#include "View/pathwidget.h"
#include "View/customscrollarea.h"
#include "View/spacewidget.h"
#include "Model/robot.h"
#include "Model/point.h"
#include <QVBoxLayout>
#include <QMainWindow>
#include <QProgressBar>
#include "mainwindow.h"
#include <QDebug>
#include "topleftmenu.h"
#include "customscrollarea.h"
#include "View/pointview.h"
#include "View/custompushbutton.h"
#include "View/stylesettings.h"


SelectedRobotWidget::SelectedRobotWidget(QWidget* parent, MainWindow* mainWindow): QWidget(parent){

    layout = new QVBoxLayout(this);

    actionButtons = new TopLeftMenu(this);

    connect(actionButtons->getEditButton(), SIGNAL(clicked()), mainWindow, SLOT(editSelecRobotBtnEvent()));

    layout->addWidget(actionButtons);

    CustomScrollArea* scrollArea = new CustomScrollArea(this);
    QVBoxLayout * inLayout = new QVBoxLayout(scrollArea);
    QWidget * inWidget = new QWidget(scrollArea);

    inWidget->setLayout(inLayout);

    name = new CustomLabel(this, true);
    inLayout->addWidget(name);

    /// Button which allow the user to scan the map from a robot
    scanBtn = new CustomPushButton(QIcon(":/icons/map.png"),"Scan a map", this, CustomPushButton::ButtonType::LEFT_MENU, "center", true);
    scanBtn->setIconSize(s_icon_size);
    inLayout->addWidget(scanBtn);
    connect(scanBtn, SIGNAL(clicked()), mainWindow, SLOT(connectToRobot()));

    /// Label which display the Ip of the robot
    ipAddressLabel = new CustomLabel("Ip : ", this);
    ipAddressLabel->setWordWrap(true);
    inLayout->addWidget(ipAddressLabel);

    /// Label which display the Wifi name of the robot
    wifiNameLabel = new CustomLabel("Wifi : ", this);
    wifiNameLabel->setWordWrap(true);
    inLayout->addWidget(wifiNameLabel);

    /// Label just for aesthetic purpose
    QLabel* batteryLabel = new QLabel("Battery Level : ", this);
    inLayout->addWidget(batteryLabel);

    /// ProgressBar which display the level of battery
    batteryLevel = new QProgressBar(this);
    batteryLevel->setValue(50);
    inLayout->addWidget(batteryLevel);


    SpaceWidget* spaceWidget2 = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    inLayout->addWidget(spaceWidget2);

    /// Home layout with the button to select/show the home
    QLabel* homeLabel = new QLabel("Home : ", this);
    homeLabel2 = new QLabel("no home set");
    homeLabel2->setAlignment(Qt::AlignCenter);
    inLayout->addWidget(homeLabel);
    inLayout->addWidget(homeLabel2);

    goHome = new CustomPushButton("Go Home", this);
    goHome->hide();

    inLayout->addWidget(goHome);

    /// Path label
    QLabel* pathLabel = new QLabel("Path : ", this);
    inLayout->addWidget(pathLabel);
    noPath = new QLabel("no path set");
    noPath->setAlignment(Qt::AlignCenter);

    inLayout->addWidget(noPath);

    pathWidget = new PathWidget(this);
    inLayout->addWidget(pathWidget);


    scrollArea->setWidget(inWidget);
    layout->addWidget(scrollArea);

    connect(goHome, SIGNAL(clicked()), mainWindow, SLOT(goHomeBtnEvent()));
    connect(actionButtons->getMapButton(), SIGNAL(clicked()), mainWindow, SLOT(checkRobotBtnEventSelect()));
    hide();
    /*setMaximumWidth(mainWindow->width()*4/10);
    setMinimumWidth(mainWindow->width()*4/10);
    inWidget->setMaximumWidth(mainWindow->width()*4/10);
    inWidget->setMinimumWidth(mainWindow->width()*4/10);*/
    inLayout->setAlignment(Qt::AlignTop);
    layout->setAlignment(Qt::AlignTop);
    layout->setContentsMargins(0,0,0,0);

}

void SelectedRobotWidget::setSelectedRobot(RobotView* const& _robotView){
    /// We update all the informations
    qDebug() << "SelectedRobotWidget::setSelectedRobot called on" << _robotView->getRobot()->getName();
    robotView = _robotView;

    name->setText(robotView->getRobot()->getName());

    batteryLevel->setValue(robotView->getRobot()->getBatteryLevel());
    ipAddressLabel->setText("Ip : "+robotView->getRobot()->getIp());
    wifiNameLabel->setText("Wifi : "+robotView->getRobot()->getWifi());

    /// If the robot has a path, we display it, otherwise we show the button to add the path
    if(robotView->getRobot()->getPath().size() > 0){
        pathWidget->setPath(robotView->getRobot()->getPath());
        pathWidget->show();
        noPath->hide();
    } else {
        pathWidget->hide();
        noPath->show();
    }

    /// If the robot has a home, we display the name of the point, otherwise a default text
    if(robotView->getRobot()->getHome() != NULL){
        homeLabel2->setText(robotView->getRobot()->getHome()->getPoint()->getName());
        goHome->show();

    } else {
        homeLabel2->setText("no home set");
        goHome->hide();
    }



    update();
}

void SelectedRobotWidget::disable(){
    scanBtn->setEnabled(false);
    goHome->setEnabled(false);
    actionButtons->setEnable(false);
}

void SelectedRobotWidget::enable(){
    scanBtn->setEnabled(true);
    goHome->setEnabled(true);
    actionButtons->setEnable(true);
}

void SelectedRobotWidget::showEvent(QShowEvent *event){
    emit showSelectedRobotWidget();
    actionButtons->disableAll();
    actionButtons->uncheckAll();
    actionButtons->getEditButton()->setEnabled(true);
    actionButtons->getMapButton()->setEnabled(true);
    actionButtons->getMapButton()->setCheckable(true);
    actionButtons->getMapButton()->setChecked(robotView->isVisible());

    QWidget::showEvent(event);
}

void SelectedRobotWidget::hideEvent(QHideEvent *event){

    emit hideSelectedRobotWidget();
    QWidget::hideEvent(event);
}

