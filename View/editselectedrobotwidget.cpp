#include "editselectedrobotwidget.h"
#include "View/robotview.h"
#include "Model/robots.h"
#include "Model/robot.h"
#include "Model/point.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QProgressBar>
#include <QLineEdit>
#include <QDebug>
#include "View/spacewidget.h"
#include <QGridLayout>
#include <pathwidget.h>
#include "verticalscrollarea.h"
EditSelectedRobotWidget::EditSelectedRobotWidget(QMainWindow* parent, const std::shared_ptr<Robots> _robots):QWidget(parent){
    robots = _robots;
    layout = new QVBoxLayout(this);
    wifiLayout = new QGridLayout();
    robotView = NULL;
    home = NULL;
    oldHome = NULL;
    pathChanged = false;


    VerticalScrollArea* scrollArea = new VerticalScrollArea(this);
    QVBoxLayout * inLayout = new QVBoxLayout(scrollArea);
    QWidget * inWidget = new QWidget(scrollArea);

    inWidget->setLayout(inLayout);


    /// Name editable label
    nameEdit = new QLineEdit(this);
    nameEdit->setStyleSheet ("text-align: left");
    inLayout->addWidget(nameEdit);

    /// Ip address label
    ipAddressLabel = new QLabel("Ip : ", this);
    ipAddressLabel->setWordWrap(true);
    inLayout->addWidget(ipAddressLabel);

    /// Wifi name label

    wifiTitle = new QLabel("Wifi : ", this);
    wifiTitle->setWordWrap(true);
    wifiTitle->setAlignment(Qt::AlignLeft);
    inLayout->addWidget(wifiTitle);

    wifiName = new QLabel("name : ", this);
    wifiName->setWordWrap(true);
    wifiName->setAlignment(Qt::AlignLeft);

    wifiNameEdit = new QLineEdit(this);
    wifiNameEdit->setStyleSheet ("text-align: left");
    wifiNameEdit->setAlignment(Qt::AlignRight);
    wifiLayout->addWidget( wifiName, 0,0);
    wifiLayout->addWidget( wifiNameEdit, 0,1);

    wifiPwd = new QLabel("Pwd : ", this);
    wifiPwd->setWordWrap(true);
    wifiPwd->setAlignment(Qt::AlignLeft);

    /// Wifi password label
    wifiPwdEdit = new QLineEdit(this);
    wifiPwdEdit->setStyleSheet ("text-align: left");
    wifiPwdEdit->setText("......");
    wifiPwdEdit->setEchoMode(QLineEdit::Password);

    wifiLayout->addWidget( wifiPwd, 1,0);
    wifiLayout->addWidget( wifiPwdEdit, 1,1);
    inLayout->addLayout(wifiLayout);

    /// Battery level widget
    QLabel* batteryLabel = new QLabel("Battery Level : ", this);
    inLayout->addWidget(batteryLabel);

    batteryLevel = new QProgressBar(this);
    batteryLevel->setValue(50);
    inLayout->addWidget(batteryLevel);

    /// Home layout with the button to select the home
    QLabel* homeLabel = new QLabel("Home : ", this);
    homeBtn = new QPushButton(QIcon(":/icons/home.png"), "", this);
    homeBtn->setIconSize(parent->size()/10);
    homeBtn->setStyleSheet ("text-align: left");
    connect(homeBtn, SIGNAL(clicked()), parent, SLOT(editHomeEvent()));
    inLayout->addWidget(homeLabel);
    inLayout->addWidget(homeBtn);

    /// Button to add a path
    addPathBtn = new QPushButton(QIcon(":/icons/plus.png"),"Add path", this);
    addPathBtn->setStyleSheet ("text-align: left");
    //addPathBtn->hide();
    addPathBtn->setIconSize(parent->size()/10);
    connect(addPathBtn, SIGNAL(clicked()), parent, SLOT(addPathSelecRobotBtnEvent()));
    inLayout->addWidget(addPathBtn);


    pathWidget = new PathWidget(this);
    inLayout->addWidget(pathWidget);


    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    inLayout->addWidget(spaceWidget);


    QHBoxLayout* grid = new QHBoxLayout();
    /// Cancel & save buttons
    cancelBtn = new QPushButton("Cancel", this);
    saveBtn = new QPushButton("Save", this);

    grid->addWidget(cancelBtn);
    grid->addWidget(saveBtn);


    connect(cancelBtn, SIGNAL(clicked()), parent, SLOT(cancelEditSelecRobotBtnEvent()));
    connect(saveBtn, SIGNAL(clicked()), this, SLOT(saveEditSelecRobotBtnEvent()));
    connect(nameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkRobotName()));
    connect(wifiNameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkWifiName()));
    connect(wifiNameEdit, SIGNAL(textEdited(QString)), this, SLOT(deletePwd()));

    hide();
    setMaximumWidth(parent->width()*4/10);
    setMinimumWidth(parent->width()*4/10);
    inWidget->setMaximumWidth(parent->width()*4/10);
    inWidget->setMinimumWidth(parent->width()*4/10);
    layout->setAlignment(Qt::AlignTop);
    layout->setContentsMargins(0,0,0,0);


    scrollArea->setWidget(inWidget);
    layout->addWidget(scrollArea);
    layout->addLayout(grid);

}

void EditSelectedRobotWidget::setSelectedRobot(RobotView* const _robotView, bool _firstConnection){

       pathChanged = false;
    firstConnection = _firstConnection;
    if(firstConnection)
        cancelBtn->setEnabled(false);
    else
        cancelBtn->setEnabled(true);

    robotView = _robotView;

    /// When a robot is selected, the informations are updated
    nameEdit->setText(robotView->getRobot()->getName());
    batteryLevel->setValue(robotView->getRobot()->getBatteryLevel());
    ipAddressLabel->setText("Ip : "+robotView->getRobot()->getIp());
    wifiNameEdit->setText(robotView->getRobot()->getWifi());

    /// If the robot has a home, we display the name of the point, otherwise a default text
    if(robotView->getRobot()->getHome() != NULL){
        homeBtn->setText(robotView->getRobot()->getHome()->getName());
        oldHome = robotView->getRobot()->getHome();
    } else {
        homeBtn->setText("Add home");
        oldHome = NULL;
    }

}

void EditSelectedRobotWidget::saveEditSelecRobotBtnEvent(void){
    qDebug() << "saveEditSelecRobotBtnEvent called";
    if(firstConnection && home == NULL){
        qDebug() << "You need to select a home";
    } else {
        emit robotSaved();
    }
}

void EditSelectedRobotWidget::checkRobotName(void){
    qDebug() << "checkRobotName called";
    if(nameEdit->text() == ""){
        saveBtn->setEnabled(false);
        qDebug() << "Error : this name is not valid," << nameEdit->text() << "can not be empty";
    } else if(robots->existRobotName(nameEdit->text()) && nameEdit->text() != robotView->getRobot()->getName()){
        saveBtn->setEnabled(false);
        qDebug() << "Error : this name is not valid," << nameEdit->text() << "already exist";
    } else {
        saveBtn->setEnabled(true);
        qDebug() << "This name is valid";
    }
}

void EditSelectedRobotWidget::checkWifiName(void){
    //qDebug() << "checkWifiName called";
    if(wifiNameEdit->text() == ""){
        saveBtn->setEnabled(false);
        qDebug() << "Save btn not enabled : " << wifiNameEdit->text() << "can not be empty";
    } else {
        saveBtn->setEnabled(true);
        qDebug() << "Save btn enabled";
    }
}

void EditSelectedRobotWidget::deletePwd(void){
    qDebug() << "deletePwd";

    wifiPwdEdit->setText("");
}

void EditSelectedRobotWidget::editName(void){
    robotView->getRobot()->setName(nameEdit->text());
    robotView->getRobot()->setWifi(wifiNameEdit->text());
    RobotView* rv = robots->getRobotViewByName(robotView->getRobot()->getName());
    if(rv != NULL){
        rv->getRobot()->setName(nameEdit->text());
        rv->getRobot()->setWifi(wifiNameEdit->text());
    } else {
        qDebug() << "editName : something unexpected happened";
    }
}

void EditSelectedRobotWidget::disableAll(void){
    nameEdit->setEnabled(false);
    homeBtn->setEnabled(false);
    saveBtn->setEnabled(false);
}

void EditSelectedRobotWidget::enableAll(void){
    nameEdit->setEnabled(true);
    homeBtn->setEnabled(true);
    saveBtn->setEnabled(true);
}

void EditSelectedRobotWidget::showEvent(QShowEvent *event){
    emit showEditSelectedRobotWidget();
    QWidget::showEvent(event);


}

void EditSelectedRobotWidget::hideEvent(QHideEvent *event){
    emit hideEditSelectedRobotWidget();
    QWidget::hideEvent(event);
}
