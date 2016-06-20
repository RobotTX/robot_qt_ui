#include "selectedrobotwidget.h"
#include "View/robotview.h"
#include "View/pathwidget.h"
#include "View/verticalscrollarea.h"
#include "Model/robot.h"
#include "Model/point.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QMainWindow>
#include <QProgressBar>

SelectedRobotWidget::SelectedRobotWidget(QMainWindow* parent){
    layout = new QVBoxLayout();

    /// Button with the name of the robot and which allow the user to return back
    /// to the last menu
    backBtn = new QPushButton(QIcon(":/icons/arrowLeft.png"),"Name");
    backBtn->setStyleSheet ("text-align: left");
    backBtn->setIconSize(parent->size()/10);
    layout->addWidget(backBtn);

    /// Button which allow the user to scan the map from a robot
    scanBtn = new QPushButton(QIcon(":/icons/map.png"),"Scan a map");
    scanBtn->setCheckable(true);
    scanBtn->setStyleSheet ("text-align: left");
    scanBtn->setIconSize(parent->size()/10);
    layout->addWidget(scanBtn);
    connect(scanBtn, SIGNAL(clicked()), parent, SLOT(connectToRobot()));

    /// Button which allow the user to edit the info of the robot
    editBtn = new QPushButton(QIcon(":/icons/edit.png"),"Edit");
    editBtn->setStyleSheet ("text-align: left");
    editBtn->setIconSize(parent->size()/10);
    layout->addWidget(editBtn);
    connect(editBtn, SIGNAL(clicked()), parent, SLOT(editSelecRobotBtnEvent()));

    /// Label which display the Ip of the robot
    ipAddressLabel = new QLabel("Ip : ");
    ipAddressLabel->setWordWrap(true);
    layout->addWidget(ipAddressLabel);

    /// Label which display the Wifi name of the robot
    wifiNameLabel = new QLabel("Wifi : ");
    wifiNameLabel->setWordWrap(true);
    layout->addWidget(wifiNameLabel);

    /// Label just for aesthetic purpose
    QLabel* batteryLabel = new QLabel("Battery Level : ");
    layout->addWidget(batteryLabel);

    /// ProgressBar which display the level of battery
    batteryLevel = new QProgressBar(this);
    batteryLevel->setValue(50);
    layout->addWidget(batteryLevel);

    /// Home layout with the button to select/show the home
    QHBoxLayout* grid = new QHBoxLayout();
    QLabel* homeLabel = new QLabel("Home : ");
    homeBtn = new QPushButton("");

    grid->addWidget(homeLabel);
    grid->addWidget(homeBtn);

    layout->addLayout(grid);

    /// Path label
    QLabel* pathLabel = new QLabel("Path : ");
    layout->addWidget(pathLabel);

    /// Button to add a path
    addPathBtn = new QPushButton(QIcon(":/icons/plus.png"),"Add path");
    addPathBtn->setStyleSheet ("text-align: left");
    addPathBtn->hide();
    addPathBtn->setIconSize(parent->size()/10);
    connect(addPathBtn, SIGNAL(clicked()), parent, SLOT(addPathSelecRobotBtnEvent()));
    layout->addWidget(addPathBtn);


    pathWidget = new PathWidget(this);

    scrollArea = new VerticalScrollArea(this);
    layout->addWidget(scrollArea);


    connect(backBtn, SIGNAL(clicked()), parent, SLOT(backSelecRobotBtnEvent()));
    connect(homeBtn, SIGNAL(clicked()), this, SLOT(homeBtnEvent()));

    hide();
    setMaximumWidth(parent->width()*4/10);
    setMinimumWidth(parent->width()*4/10);
    layout->setAlignment(Qt::AlignTop);
    setLayout(layout);
}

SelectedRobotWidget::~SelectedRobotWidget(){
    delete layout;
    delete batteryLevel;
    delete backBtn;
    delete wifiNameLabel;
    delete addPathBtn;
    delete ipAddressLabel;
    delete homeBtn;
    delete pathWidget;
    delete scrollArea;
    delete robotView;
    delete scanBtn;
    delete editBtn;
}

void SelectedRobotWidget::setSelectedRobot(RobotView* const& _robotView){
    /// We update all the informations
    robotView = _robotView;
    backBtn->setText(robotView->getRobot()->getName());
    batteryLevel->setValue(robotView->getRobot()->getBatteryLevel());
    ipAddressLabel->setText("Ip : "+robotView->getRobot()->getIp());
    wifiNameLabel->setText("Wifi : "+robotView->getRobot()->getWifi());

    /// If the robot has a path, we display it, otherwise we show the button to add the path
    if(robotView->getRobot()->getPath().size() > 0){
        addPathBtn->hide();
        pathWidget->setSelectedRobot(robotView);
        scrollArea->show();
    } else {
        addPathBtn->show();
        scrollArea->hide();
    }

    /// If the robot has a home, we display the name of the point, otherwise a default text
    if(robotView->getRobot()->getHome() != NULL){
        homeBtn->setText(robotView->getRobot()->getHome()->getName());
    } else {
        homeBtn->setText("Select a home");
    }


    scrollArea->setWidget(pathWidget);

    update();
}

void SelectedRobotWidget::homeBtnEvent(){
    if(robotView->getRobot()->getHome() != NULL){
        emit showHome(robotView);
    } else {
        emit selectHome(robotView);
    }
}

void SelectedRobotWidget::disable(){
    backBtn->setEnabled(false);
    editBtn->setEnabled(false);
    homeBtn->setEnabled(false);
    addPathBtn->setEnabled(false);
}

void SelectedRobotWidget::enable(){
    backBtn->setEnabled(true);
    editBtn->setEnabled(true);
    homeBtn->setEnabled(true);
    addPathBtn->setEnabled(true);
}
