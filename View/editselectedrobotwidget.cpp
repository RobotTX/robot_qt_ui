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

EditSelectedRobotWidget::EditSelectedRobotWidget(QMainWindow* parent, const std::shared_ptr<Robots> _robots){
    robots = _robots;
    layout = new QVBoxLayout();
   wifiLayout = new QGridLayout;
//   QFormLayout *wifiLayout = new QFormLayout;

    /// Nale editable label
    nameEdit = new QLineEdit(parent);
    nameEdit->setStyleSheet ("text-align: left");
    layout->addWidget(nameEdit);

    /// Ip address label
    ipAddressLabel = new QLabel("Ip : ");
    ipAddressLabel->setWordWrap(true);
    layout->addWidget(ipAddressLabel);

    /// Wifi name label

    wifiTitle = new QLabel("Wifi : ");
    wifiTitle->setWordWrap(true);
    wifiTitle->setAlignment(Qt::AlignLeft);
    layout->addWidget(wifiTitle);

    wifiName = new QLabel("name : ");
    wifiName->setWordWrap(true);
    wifiName->setAlignment(Qt::AlignLeft);

    wifiNameEdit = new QLineEdit(parent);
    wifiNameEdit->setStyleSheet ("text-align: left");
    wifiNameEdit->setAlignment(Qt::AlignRight);
    wifiLayout->addWidget( wifiName, 0,0);
    wifiLayout->addWidget( wifiNameEdit, 0,1);


    wifiPwd = new QLabel("Pwd : ");
    wifiPwd->setWordWrap(true);
    wifiPwd->setAlignment(Qt::AlignLeft);

    /// Wifi password label
    wifiPwdEdit = new QLineEdit(parent);
    wifiPwdEdit->setStyleSheet ("text-align: left");
    wifiPwdEdit->setText("......");
    wifiPwdEdit->setEchoMode(QLineEdit::Password);

    wifiLayout->addWidget( wifiPwd, 1,0);
    wifiLayout->addWidget( wifiPwdEdit, 1,1);
    layout->addLayout(wifiLayout);


    /// Battery level widget
    QLabel* batteryLabel = new QLabel("Battery Level : ");
    layout->addWidget(batteryLabel);

    batteryLevel = new QProgressBar(this);
    batteryLevel->setValue(50);
    layout->addWidget(batteryLevel);

    /// Path label
    QLabel* pathLabel = new QLabel("Path : ");
    layout->addWidget(pathLabel);

    /// Home layout with the button to select the home
    QLabel* homeLabel = new QLabel("Home : ");
    homeBtn = new QPushButton(QIcon(":/icons/home.png"), "");
    homeBtn->setIconSize(parent->size()/10);
    homeBtn->setStyleSheet ("text-align: left");
    connect(homeBtn, SIGNAL(clicked()), parent, SLOT(editHomeEvent()));

    layout->addWidget(homeLabel);
    layout->addWidget(homeBtn);

    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL);
    layout->addWidget(spaceWidget);

    QHBoxLayout* grid = new QHBoxLayout();
    /// Cancel & save buttons
    QPushButton* cancelBtn = new QPushButton("Cancel");
    saveBtn = new QPushButton("Save");

    grid->addWidget(cancelBtn);
    grid->addWidget(saveBtn);

    layout->addLayout(grid);

    connect(cancelBtn, SIGNAL(clicked()), parent, SLOT(cancelEditSelecRobotBtnEvent()));
    connect(saveBtn, SIGNAL(clicked()), this, SLOT(saveEditSelecRobotBtnEvent()));
    connect(nameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkRobotName()));
    connect(wifiNameEdit, SIGNAL(textEdited(QString)), this, SLOT(deletePwd()));



    hide();
    setMaximumWidth(parent->width()*4/10);
    setMinimumWidth(parent->width()*4/10);
    layout->setAlignment(Qt::AlignTop );

    setLayout(layout);
}

EditSelectedRobotWidget::~EditSelectedRobotWidget(){
    delete layout;
    delete wifiLayout;
    delete batteryLevel;
    delete nameEdit;
    delete wifiTitle;
    delete wifiName;
    delete wifiNameEdit;
    delete wifiPwd;
    delete wifiPwdEdit;
    delete addPathBtn;
    delete ipAddressLabel;
    delete saveBtn;
    delete homeBtn;
}

void EditSelectedRobotWidget::setSelectedRobot(RobotView* const _robotView){
    robotView = _robotView;
    /// When a robot is selected, the informations are updated
    nameEdit->setText(robotView->getRobot()->getName());
    batteryLevel->setValue(robotView->getRobot()->getBatteryLevel());
    ipAddressLabel->setText("Ip : "+robotView->getRobot()->getIp());
    wifiNameEdit->setText(robotView->getRobot()->getWifi());
    home = NULL;

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
    /// Save the change on the model
    emit robotSaved();
}

void EditSelectedRobotWidget::checkRobotName(void){
    qDebug() << "checkRobotName called";
    if((robots->existRobotName(nameEdit->text()) || nameEdit->text() == "") && nameEdit->text() != robotView->getRobot()->getName()){
        saveBtn->setEnabled(false);
        qDebug() << "Save btn not enabled : " << nameEdit->text() << "already exist";
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

    robots->getRobotViewByName(robotView->getRobot()->getName())->getRobot()->setName(nameEdit->text());
    robots->getRobotViewByName(robotView->getRobot()->getName())->getRobot()->setWifi(wifiNameEdit->text());

}

void EditSelectedRobotWidget::disableAll(void){
    nameEdit->setEnabled(false);
    homeBtn->setEnabled(false);
    //addPathBtn->setEnabled(false);
    saveBtn->setEnabled(false);
}

void EditSelectedRobotWidget::enableAll(void){
    nameEdit->setEnabled(true);
    homeBtn->setEnabled(true);
    //addPathBtn->setEnabled(true);
    saveBtn->setEnabled(true);
}
