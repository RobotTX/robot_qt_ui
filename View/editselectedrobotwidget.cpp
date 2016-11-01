#include "editselectedrobotwidget.h"
#include "View/robotview.h"
#include "Model/robots.h"
#include "Model/robot.h"
#include "Model/point.h"
#include <QVBoxLayout>
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
#include <QMenu>
#include <assert.h>
#include "View/customlabel.h"
#include "Model/points.h"
#include <QProgressBar>
#include "View/customlabel.h"
#include "View/customrobotdialog.h"
#include "View/customlineedit.h"


EditSelectedRobotWidget::EditSelectedRobotWidget(QWidget* parent, MainWindow* _mainWindow, const QSharedPointer<Points> &_points, const QSharedPointer<Robots> _robots, const QSharedPointer<Paths> &_paths):
    QWidget(parent), mainWindow(_mainWindow), points(_points), robots(_robots), paths(_paths), assignedPath("")
{
    /// creates a dialog widget to modify the robot's info and centers it on the main window
    robotDialog = new CustomRobotDialog(this);

    connect(robotDialog->getCancelButton(), SIGNAL(clicked()), this, SLOT(cancelRobotModifications()));
    connect(robotDialog->getSaveButton(), SIGNAL(clicked()), mainWindow, SLOT(saveRobotModifications()));
    connect(this, SIGNAL(sendPathSelectedRobot()), mainWindow, SLOT(sendPathSelectedRobotSlot()));

    layout = new QVBoxLayout(this);
    robotView = NULL;
    home = QSharedPointer<PointView>();
    pathChanged = false;
    editing = false;

    CustomScrollArea* scrollArea = new CustomScrollArea(this, true, true);
    QVBoxLayout * inLayout = new QVBoxLayout(scrollArea);
    inWidget = new QWidget(scrollArea);

    inWidget->setLayout(inLayout);

    /// Name editable label
    nameLabel = new CustomLabel(this, true);
    inLayout->addWidget(nameLabel);

    /// Button which allow the user to scan the map from a robot
    scanBtn = new CustomPushButton(QIcon(":/icons/map.png"),"Scan a map", this, true, CustomPushButton::ButtonType::LEFT_MENU, "center", true);
    scanBtn->setIconSize(s_icon_size);
    inLayout->addWidget(scanBtn);
    connect(scanBtn, SIGNAL(clicked(bool)), mainWindow, SLOT(connectToRobot(bool)));

    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    inLayout->addWidget(spaceWidget);

    /// Ip address label
    ipAddressLabel = new QLabel("Ip : ", this);
    ipAddressLabel->setWordWrap(true);
    inLayout->addWidget(ipAddressLabel);

    /// Wifi name label

    QLabel* wifiTitle = new QLabel("Wifi name :", this);
    wifiTitle->setWordWrap(true);
    //wifiTitle->setAlignment(Qt::AlignLeft);
    inLayout->addWidget(wifiTitle);

    wifiNameLabel = new CustomLabel(this);
    inLayout->addWidget(wifiNameLabel);

    QLabel* wifiPwd = new QLabel("Wifi pwd :", this);
    //wifiPwd->setAlignment(Qt::AlignLeft);

    inLayout->addWidget(wifiPwd);

    /// ProgressBar which display the level of battery
    batteryLevel = new QProgressBar(this);
    batteryLevel->setValue(50);
    inLayout->addWidget(batteryLevel);

    editRobotInfoBtn = new CustomPushButton(QIcon(":/icons/edit.png") , "Edit information", this, true);
    editRobotInfoBtn->setIconSize(xs_icon_size);
    inLayout->addWidget(editRobotInfoBtn);
    connect(editRobotInfoBtn, SIGNAL(clicked(bool)), this, SLOT(editRobot()));

    SpaceWidget* spaceWidget2 = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    inLayout->addWidget(spaceWidget2);

    /// Home layout with the button to select the home
    homeLabel = new CustomLabel("Home : ", this);
    homeBtn = new CustomPushButton(QIcon(":/icons/home.png"), "Assign a home", this);
    homeBtn->setIconSize(s_icon_size);
    connect(homeBtn, SIGNAL(clicked(bool)), this, SLOT(openHomeMenu()));
    inLayout->addWidget(homeLabel);
    inLayout->addWidget(homeBtn);

    /// to assign an existing point to be the home of the robot
    homeMenu = new QMenu("Assign home", this);
    updateHomeMenu();
    connect(homeMenu, SIGNAL(triggered(QAction*)), this, SLOT(assignHome(QAction*)));

    goHomeBtn = new CustomPushButton(QIcon(":/icons/play.png"), "Go home", this);
    goHomeBtn->setToolTip("Clicking this button will send the robot to its home");
    goHomeBtn->setIconSize(s_icon_size);
    goHomeBtn->hide();
    connect(goHomeBtn, SIGNAL(clicked()), mainWindow, SLOT(goHome()));
    inLayout->addWidget(goHomeBtn);

    /// to assign an existing path to the robot
    assignPathButton = new CustomPushButton(QIcon(":/icons/path.png"), "Assign a path", this);
    assignPathButton->setIconSize(s_icon_size);
    inLayout->addWidget(assignPathButton);
    connect(assignPathButton, SIGNAL(clicked(bool)), this, SLOT(openMenu()));

    pathsMenu = new QMenu("Assign path", this);
    updatePathsMenu();
    connect(pathsMenu, SIGNAL(triggered(QAction*)), this, SLOT(assignPath(QAction*)));

    deletePathBtn = new CustomPushButton(QIcon(":/icons/empty.png"),"Delete Path", this);
    deletePathBtn->setIconSize(s_icon_size);
    deletePathBtn->hide();
    connect(deletePathBtn, SIGNAL(clicked()), mainWindow, SLOT(deletePathSelecRobotBtnEvent()));
    inLayout->addWidget(deletePathBtn);

    pathSpaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    inLayout->addWidget(pathSpaceWidget);

    pathWidget = new PathWidget(this);
    inLayout->addWidget(pathWidget);


    /// to display a path that's assigned to the robot after clearing the map of other path(s)
    connect(this, SIGNAL(clearMapOfPaths()), mainWindow, SLOT(clearMapOfPaths()));
    connect(this, SIGNAL(showPath(QString, QString)), mainWindow, SLOT(displayAssignedPath(QString, QString)));

    connect(this, SIGNAL(newHome(QString)), mainWindow, SLOT(setNewHome(QString)));

    hide();

    inLayout->setAlignment(Qt::AlignTop);
    inLayout->setContentsMargins(0, 0, 10, 0);
    layout->setContentsMargins(0, 0, 0, 0);

    scrollArea->setWidget(inWidget);
    layout->addWidget(scrollArea);
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
}

void EditSelectedRobotWidget::setSelectedRobot(RobotView* const _robotView, bool _firstConnection){
    qDebug() << _robotView->getRobot()->getName();

    pathChanged = false;
    firstConnection = _firstConnection;
    /*
    if(firstConnection)
        cancelBtn->setEnabled(false);
    else
        cancelBtn->setEnabled(true);
*/
    robotView = _robotView;

    /// When a robot is selected, the informations are updated
    nameLabel->setText(robotView->getRobot()->getName());
    ipAddressLabel->setText("Ip : "+robotView->getRobot()->getIp());
    wifiNameLabel->setText(robotView->getRobot()->getWifi());

    /// If the robot has a home, we display the name of the point, otherwise a default text
    if(robotView->getRobot()->getHome() != NULL){
        home = robotView->getRobot()->getHome();
        homeLabel->setText("Home : "+robotView->getRobot()->getHome()->getPoint()->getName());

    } else {
        homeBtn->setText("Assign a home point");
        homeLabel->setText("Home : ");
        home = QSharedPointer<PointView>();
    }
}

void EditSelectedRobotWidget::editName(void){
    robotView->getRobot()->setName(nameLabel->text());
    robotView->getRobot()->setWifi(wifiNameLabel->text());
    RobotView* rv = robots->getRobotViewByName(robotView->getRobot()->getName());
    if(rv != NULL){
        rv->getRobot()->setName(nameLabel->text());
        rv->getRobot()->setWifi(wifiNameLabel->text());
    } else {
        qDebug() << "editName : something unexpected happened";
    }
}

void EditSelectedRobotWidget::setEnableAll(const bool enable){
    qDebug() << "EditSelectedRobotWidget::setEnableAll called" << enable;
    homeBtn->setEnabled(enable);
    deletePathBtn->setEnabled(enable);
    assignPathButton->setEnabled(enable);
    editRobotInfoBtn->setEnabled(enable);
    goHomeBtn->setEnabled(enable);
}

void EditSelectedRobotWidget::showEvent(QShowEvent *event){
    setEnableAll(true);
    emit showEditSelectedRobotWidget();
    updatePathsMenu();
    updateHomeMenu();
    pathWidget->setPath(robotView->getRobot()->getPath());
    if(robotView->getRobot()->getPath().size() < 1)
        pathWidget->hide();
    else
        pathWidget->show();
    QWidget::showEvent(event);
}

void EditSelectedRobotWidget::hideEvent(QHideEvent *event){
    emit hideEditSelectedRobotWidget();
    QWidget::hideEvent(event);
}

void EditSelectedRobotWidget::setPath(const QVector<QSharedPointer<PathPoint> >& path){
    if(path.size() > 0){
        pathWidget->show();
        pathSpaceWidget->show();
        deletePathBtn->show();
        pathWidget->setPath(path);
    }
}

void EditSelectedRobotWidget::clearPath(){
    pathWidget->hide();
    pathSpaceWidget->hide();
    deletePathBtn->hide();
    assignedPath = "";
    groupAssignedPath = "";
}

void EditSelectedRobotWidget::updatePathsMenu(){
    //qDebug() << "updatePathsMenu called";
    pathsMenu->clear();
    QMapIterator<QString, QSharedPointer<Paths::CollectionPaths>> i(*(paths->getGroups()));
    while (i.hasNext()) {
        i.next();
        QMapIterator<QString, QSharedPointer<Paths::Path> > it_paths(*(i.value()));
        QMenu* group = pathsMenu->addMenu("&" + i.key());
        while(it_paths.hasNext()){
            it_paths.next();
            if(it_paths.value()){
                group->addAction(it_paths.key());
                if(!assignedPath.compare(it_paths.key()) && !groupAssignedPath.compare(i.key())){
                    group->actions().last()->setCheckable(true);
                    group->actions().last()->setChecked(true);
                }
            }
        }
    }
}

void EditSelectedRobotWidget::openMenu(){
    pathsMenu->exec(QCursor::pos());
}

void EditSelectedRobotWidget::assignPath(QAction *action){
    emit clearMapOfPaths();
    setPathChanged(true);
    action->setCheckable(true);
    action->setChecked(true);
    bool foundFlag(false);
    QString groupName = static_cast<QMenu*> (action->associatedWidgets().at(0))->title().mid(1);
    assignedPath = action->text();
    groupAssignedPath = groupName;
    setPath(paths->getPath(groupName, action->text(), foundFlag));
    assert(foundFlag);
    emit showPath(groupName, action->text());
    emit sendPathSelectedRobot();
}

void EditSelectedRobotWidget::openHomeMenu(){
    homeMenu->exec(QCursor::pos());
}

void EditSelectedRobotWidget::assignHome(QAction *action){
    action->setCheckable(true);
    action->setChecked(true);
    QString homeName = action->text();
    homeLabel->setText("Home : " + homeName);
    homeLabel->wordWrap();
    action->setEnabled(false);
    emit newHome(homeName);
}

void EditSelectedRobotWidget::updateHomeMenu(){
    homeMenu->clear();
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();
        if(i.value() && i.key().compare(TMP_GROUP_NAME) && i.key().compare(PATH_GROUP_NAME)){
            if(i.value()->count() > 0){
                QMenu *group = homeMenu->addMenu("&" + i.key());
                for(int j = 0; j < i.value()->count(); j++){
                    group->addAction(i.value()->at(j)->getPoint()->getName());
                    if(i.value()->at(j)->getPoint()->isHome()){
                        group->actions().at(j)->setEnabled(false);
                        if(!i.value()->at(j)->getPoint()->getRobotName().compare(robotView->getRobot()->getName())){
                            group->actions().at(j)->setCheckable(true);
                            group->actions().at(j)->setChecked(true);
                        }
                    }
                }
            }
        }
    }
}

void EditSelectedRobotWidget::editRobot(){
    qDebug() << "EditSelectedRobotWidget::editRobot called";
    robotDialog->move(mainWindow->pos().x() + mainWindow->width()/2-robotDialog->width()/2,
                      mainWindow->pos().y() + mainWindow->height()/2-robotDialog->height()/2);
    robotDialog->getNameEdit()->setText(robotView->getRobot()->getName());
    robotDialog->getSSIDEdit()->setText(robotView->getRobot()->getWifi());
    robotDialog->getPasswordEdit()->setText("......");
    robotDialog->exec();
}

void EditSelectedRobotWidget::cancelRobotModifications(){
    qDebug() << "EditSelectedRobotWidget::cancelRobotModifications called";
    robotDialog->getNameEdit()->setText(robotView->getRobot()->getName());
    robotDialog->getSSIDEdit()->setText(robotView->getRobot()->getWifi());
    robotDialog->getPasswordEdit()->setText("......");
    robotDialog->close();
}

void EditSelectedRobotWidget::resizeEvent(QResizeEvent *event){
    QWidget* widget = static_cast<QWidget*>(parent());
    int maxWidth = widget->width() - 10;
    setFixedWidth(maxWidth);
    QWidget::resizeEvent(event);
}
