#include "editselectedrobotwidget.h"
#include "View/robotview.h"
#include "Model/robots.h"
#include "Model/robot.h"
#include "Model/point.h"
#include <QVBoxLayout>
#include <QLabel>
#include "Controller/mainwindow.h"
#include <QHBoxLayout>
#include <QProgressBar>
#include <QLineEdit>
#include <QDebug>
#include "View/spacewidget.h"
#include <QGridLayout>
#include <pathwidget.h>
#include "customscrollarea.h"
#include "View/custompushbutton.h"
#include "View/stylesettings.h"
#include "View/customlineedit.h"


EditSelectedRobotWidget::EditSelectedRobotWidget(QWidget* parent, MainWindow* mainWindow, const QSharedPointer<Robots> _robots):QWidget(parent){
    robots = _robots;
    layout = new QVBoxLayout(this);
    wifiLayout = new QGridLayout();
    robotView = NULL;
    home = QSharedPointer<PointView>();
    oldHome = QSharedPointer<PointView>();
    pathChanged = false;
    editing = false;


    CustomScrollArea* scrollArea = new CustomScrollArea(this);
    QVBoxLayout * inLayout = new QVBoxLayout(scrollArea);
    QWidget * inWidget = new QWidget(scrollArea);

    inWidget->setLayout(inLayout);


    /// Name editable label
    nameEdit = new CustomLineEdit(this);
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

    wifiNameEdit = new CustomLineEdit(this);
    wifiNameEdit->setAlignment(Qt::AlignRight);
    wifiLayout->addWidget( wifiName, 0,0);
    wifiLayout->addWidget( wifiNameEdit, 0,1);

    wifiPwd = new QLabel("Pwd : ", this);
    wifiPwd->setWordWrap(true);
    wifiPwd->setAlignment(Qt::AlignLeft);

    /// Wifi password label
    wifiPwdEdit = new CustomLineEdit(this);
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
     homeLabel = new QLabel("Home : ", this);
    homeBtn = new CustomPushButton(QIcon(":/icons/home.png"), "Add Home", this);
    homeBtn->setIconSize(s_icon_size);
    connect(homeBtn, SIGNAL(clicked()), mainWindow, SLOT(editHomeEvent()));
    inLayout->addWidget(homeLabel);
    inLayout->addWidget(homeBtn);

    /// Button to add a path
    addPathBtn = new CustomPushButton(QIcon(":/icons/plus.png"),"Add Path", this);
    addPathBtn->setIconSize(xs_icon_size);
    connect(addPathBtn, SIGNAL(clicked()), mainWindow, SLOT(addPathSelecRobotBtnEvent()));
    inLayout->addWidget(addPathBtn);

    deletePathBtn = new CustomPushButton(QIcon(":/icons/empty.png"),"Delete Path", this);
    deletePathBtn->setIconSize(s_icon_size);
    deletePathBtn->hide();
    connect(deletePathBtn, SIGNAL(clicked()), mainWindow, SLOT(deletePathSelecRobotBtnEvent()));
    inLayout->addWidget(deletePathBtn);


    pathWidget = new PathWidget(this);
    inLayout->addWidget(pathWidget);


    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    inLayout->addWidget(spaceWidget);


    QHBoxLayout* grid = new QHBoxLayout();
    /// Cancel & save buttons
    cancelBtn = new CustomPushButton("Cancel", this, CustomPushButton::ButtonType::LEFT_MENU, "center");
    saveBtn = new CustomPushButton("Save", this, CustomPushButton::ButtonType::LEFT_MENU, "center");

    grid->addWidget(cancelBtn);
    grid->addWidget(saveBtn);


    connect(cancelBtn, SIGNAL(clicked()), mainWindow, SLOT(cancelEditSelecRobotBtnEvent()));
    connect(saveBtn, SIGNAL(clicked()), this, SLOT(saveEditSelecRobotBtnEvent()));
    connect(nameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkRobotName()));
    connect(wifiNameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkWifiName()));
    connect(wifiNameEdit, SIGNAL(textEdited(QString)), this, SLOT(deletePwd()));

    hide();
    /*setMaximumWidth(mainWindow->width()*4/10);
    setMinimumWidth(mainWindow->width()*4/10);
    inWidget->setMaximumWidth(mainWindow->width()*4/10);
    inWidget->setMinimumWidth(mainWindow->width()*4/10);*/
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
        homeBtn->setText("Edit Home");
        oldHome = robotView->getRobot()->getHome();
        homeLabel->setText("Home: "+robotView->getRobot()->getHome()->getPoint()->getName());

    } else {
        homeBtn->setText("Add Home");
        homeLabel->setText("Home: ");
        oldHome = QSharedPointer<PointView>();
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

void EditSelectedRobotWidget::setEnableAll(bool enable){
    nameEdit->setEnabled(enable);
    homeBtn->setEnabled(enable);
    saveBtn->setEnabled(enable);
    addPathBtn->setEnabled(enable);
    deletePathBtn->setEnabled(enable);
    wifiNameEdit->setEnabled(enable);
    wifiPwdEdit->setEnabled(enable);
    nameEdit->setEnabled(enable);
}

void EditSelectedRobotWidget::showEvent(QShowEvent *event){
    setEnableAll(true);
    emit showEditSelectedRobotWidget();
    QWidget::showEvent(event);
}

void EditSelectedRobotWidget::hideEvent(QHideEvent *event){
    emit hideEditSelectedRobotWidget();
    QWidget::hideEvent(event);
}

void EditSelectedRobotWidget::setPath(const QVector<QSharedPointer<PathPoint> >& path){
    if(path.size() > 0){
        pathWidget->show();
        addPathBtn->setText("Edit Path");
        addPathBtn->setIcon(QIcon(":/icons/edit.png"));
        deletePathBtn->show();
        pathWidget->setPath(path);
    }
}

void EditSelectedRobotWidget::clearPath(){
    addPathBtn->setText("Add Path");
    addPathBtn->setIcon(QIcon(":/icons/plus.png"));
    addPathBtn->setIconSize(xs_icon_size);
    pathWidget->hide();
    deletePathBtn->hide();
}
