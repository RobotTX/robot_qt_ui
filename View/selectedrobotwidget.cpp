#include "selectedrobotwidget.h"
#include "View/robotview.h"
#include "View/pathwidget.h"
#include "View/customscrollarea.h"
#include "View/spacewidget.h"
#include "Model/robot.h"
#include "Model/point.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QMainWindow>
#include <QProgressBar>
#include "mainwindow.h"
#include <QDebug>
#include "topleftmenu.h"
#include "customscrollarea.h"
#include "buttonmenu.h"
#include "View/pointview.h"


SelectedRobotWidget::SelectedRobotWidget(QMainWindow* parent): QWidget(parent){


    layout = new QVBoxLayout(this);

    actionButtons = new TopLeftMenu(this);

    connect(actionButtons->getEditButton(), SIGNAL(clicked()), parent, SLOT(editSelecRobotBtnEvent()));


    layout->addWidget(actionButtons);

    CustomScrollArea* scrollArea = new CustomScrollArea(this);
    QVBoxLayout * inLayout = new QVBoxLayout(scrollArea);
    QWidget * inWidget = new QWidget(scrollArea);

    inWidget->setLayout(inLayout);

    name = new QLabel();
    name->setAlignment(Qt::AlignCenter);
    name->setStyleSheet("font-weight: bold; text-decoration:underline");
    inLayout->addWidget(name);

    /// Button which allow the user to scan the map from a robot
    scanBtn = new QPushButton(QIcon(":/icons/map.png"),"Scan a map", this);
    scanBtn->setCheckable(true);
    scanBtn->setStyleSheet ("text-align: left");
    scanBtn->setIconSize(parent->size()/10);
    inLayout->addWidget(scanBtn);
    connect(scanBtn, SIGNAL(clicked()), parent, SLOT(connectToRobot()));

    /// Label which display the Ip of the robot
    ipAddressLabel = new QLabel("Ip : ", this);
    ipAddressLabel->setWordWrap(true);
    inLayout->addWidget(ipAddressLabel);

    /// Label which display the Wifi name of the robot
    wifiNameLabel = new QLabel("Wifi : ", this);
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

    /*
    homeBtn = new QPushButton(QIcon(":/icons/home.png"), "", this);
    homeBtn->setIconSize(parent->size()/10);
    homeBtn->setStyleSheet ("text-align: left");
    */
    inLayout->addWidget(homeLabel);
    inLayout->addWidget(homeLabel2);

    goHome = new QPushButton("Go Home", this);
    goHome->setMinimumHeight(30);
    goHome->setMaximumHeight(30);
    goHome->hide();

    inLayout->addWidget(goHome);

    /// Path label
    QLabel* pathLabel = new QLabel("Path : ", this);
    inLayout->addWidget(pathLabel);
    noPath = new QLabel("no path set");
    inLayout->addWidget(noPath);

/*
    /// Button to add a path
    addPathBtn = new QPushButton(QIcon(":/icons/plus.png"),"Add path", this);
    addPathBtn->setStyleSheet ("text-align: left");
    addPathBtn->hide();
    addPathBtn->setIconSize(parent->size()/10);
    connect(addPathBtn, SIGNAL(clicked()), parent, SLOT(addPathSelecRobotBtnEvent()));
    inLayout->addWidget(addPathBtn);
*/

    pathWidget = new PathWidget(this);
    inLayout->addWidget(pathWidget);


    scrollArea->setWidget(inWidget);
    layout->addWidget(scrollArea);

  //  connect(homeBtn, SIGNAL(clicked()), this, SLOT(homeBtnEvent()));
    connect(goHome, SIGNAL(clicked()), parent, SLOT(goHomeBtnEvent()));
    connect(actionButtons->getMapButton(), SIGNAL(clicked()), parent, SLOT(checkRobotBtnEventSelect()));
    hide();
    setMaximumWidth(parent->width()*4/10);
    setMinimumWidth(parent->width()*4/10);
    inWidget->setMaximumWidth(parent->width()*4/10);
    inWidget->setMinimumWidth(parent->width()*4/10);
    layout->setAlignment(Qt::AlignTop);
    layout->setContentsMargins(0,0,0,0);

}

void SelectedRobotWidget::setSelectedRobot(RobotView* const& _robotView){
    /// We update all the informations
    qDebug() << "select this robot";
    robotView = _robotView;

    name->setText(robotView->getRobot()->getName());

    batteryLevel->setValue(robotView->getRobot()->getBatteryLevel());
    ipAddressLabel->setText("Ip : "+robotView->getRobot()->getIp());
    wifiNameLabel->setText("Wifi : "+robotView->getRobot()->getWifi());

    /// If the robot has a path, we display it, otherwise we show the button to add the path
    if(robotView->getRobot()->getPath().size() > 0){
       // addPathBtn->hide();
        pathWidget->setSelectedRobot(robotView);
        pathWidget->show();
        noPath->hide();
    } else {
       // addPathBtn->show();
        pathWidget->hide();
        noPath->show();
    }

    /// If the robot has a home, we display the name of the point, otherwise a default text
    if(robotView->getRobot()->getHome() != NULL){
        homeLabel2->setText(robotView->getRobot()->getHome()->getPoint()->getName());
        goHome->show();

    } else {
      /*  homeBtn->setText("Add home");
        homeBtn->setFlat(false);
        //homeBtn->setStyleSheet ("QPushButton[enabled=\"false\"] {color: lightgrey;}");
        homeBtn->setEnabled(true);*/
        homeLabel2->setText("no home set");
        goHome->hide();
    }


    update();
}

void SelectedRobotWidget::homeBtnEvent(){
    if(robotView->getRobot()->getHome() == NULL){
        emit selectHome(robotView);
    }
}

void SelectedRobotWidget::disable(){
   // editBtn->setEnabled(false);
    //homeBtn->setEnabled(false);
   // addPathBtn->setEnabled(false);
   scanBtn->setEnabled(false);

    goHome->setEnabled(false);
    actionButtons->setEnable(false);
}

void SelectedRobotWidget::enable(){
  //  editBtn->setEnabled(true);
   // addPathBtn->setEnabled(true);

    scanBtn->setEnabled(true);
    goHome->setEnabled(true);
/*
    if(robotView->getRobot()->getHome() != NULL){
        homeBtn->setEnabled(false);
    } else {
        homeBtn->setEnabled(true);
    }*/
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

QString SelectedRobotWidget::getName(void)
{
    return name->text();
}

