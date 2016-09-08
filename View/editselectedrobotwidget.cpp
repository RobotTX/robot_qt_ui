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
#include <QMenu>
#include <assert.h>
#include "View/customlineedit.h"
#include "Model/points.h"


EditSelectedRobotWidget::EditSelectedRobotWidget(QWidget* parent, MainWindow* mainWindow, const QSharedPointer<Points> &_points, const QSharedPointer<Robots> _robots, const QSharedPointer<Paths> &_paths):
    QWidget(parent), points(_points), robots(_robots), paths(_paths), assignedPath("")
{
    layout = new QVBoxLayout(this);
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


    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    inLayout->addWidget(spaceWidget);

    /// Ip address label
    ipAddressLabel = new QLabel("Ip : ", this);
    ipAddressLabel->setWordWrap(true);
    inLayout->addWidget(ipAddressLabel);

    /// Wifi name label

    QLabel* wifiTitle = new QLabel("Wifi name :", this);
    wifiTitle->setWordWrap(true);
    wifiTitle->setAlignment(Qt::AlignLeft);
    inLayout->addWidget(wifiTitle);

    wifiNameEdit = new CustomLineEdit(this);
    inLayout->addWidget(wifiNameEdit);

    QLabel* wifiPwd = new QLabel("Wifi pwd :", this);
    wifiPwd->setAlignment(Qt::AlignLeft);

    /// Wifi password label
    wifiPwdEdit = new CustomLineEdit(this);
    wifiPwdEdit->setText("......");
    wifiPwdEdit->setEchoMode(QLineEdit::Password);

    inLayout->addWidget(wifiPwd);
    inLayout->addWidget(wifiPwdEdit);


    SpaceWidget* spaceWidget2 = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    inLayout->addWidget(spaceWidget2);

    /// Home layout with the button to select the home
    homeLabel = new QLabel("Home : ", this);
    homeBtn = new CustomPushButton(QIcon(":/icons/home.png"), "Assign a home point", this);
    homeBtn->setIconSize(s_icon_size);
    connect(homeBtn, SIGNAL(clicked(bool)), this, SLOT(openHomeMenu()));
    //connect(homeBtn, SIGNAL(clicked()), mainWindow, SLOT(editHomeEvent()));
    inLayout->addWidget(homeLabel);
    inLayout->addWidget(homeBtn);

    /// to assign an existing point to be the home of the robot
    homeMenu = new QMenu("Assign home", this);
    updateHomeMenu();
    connect(homeMenu, SIGNAL(triggered(QAction*)), this, SLOT(assignHome(QAction*)));

    /// Button to add a path
    addPathBtn = new CustomPushButton(QIcon(":/icons/plus.png"),"Create Path", this);
    addPathBtn->setIconSize(xs_icon_size);
    connect(addPathBtn, SIGNAL(clicked()), mainWindow, SLOT(addPathSelecRobotBtnEvent()));
    inLayout->addWidget(addPathBtn);

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


    QHBoxLayout* grid = new QHBoxLayout();
    /// Cancel & save buttons
    cancelBtn = new CustomPushButton("Cancel", this, CustomPushButton::ButtonType::LEFT_MENU, "center");
    cancelBtn->setToolTip("This will discard all changes made since the last time you saved");
    saveBtn = new CustomPushButton("Save", this, CustomPushButton::ButtonType::LEFT_MENU, "center");
    saveBtn->setToolTip("This will apply the your changes to the robot");

    grid->addWidget(cancelBtn);
    grid->addWidget(saveBtn);


    connect(cancelBtn, SIGNAL(clicked()), mainWindow, SLOT(cancelEditSelecRobotBtnEvent()));
    connect(saveBtn, SIGNAL(clicked()), this, SLOT(saveEditSelecRobotBtnEvent()));
    connect(nameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkRobotName()));
    connect(wifiNameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkWifiName()));
    connect(wifiNameEdit, SIGNAL(textEdited(QString)), this, SLOT(deletePwd()));

    /// to display a path that's assigned to the robot after clearing the map of other path(s)
    connect(this, SIGNAL(clearMapOfPaths()), mainWindow, SLOT(clearMapOfPaths()));
    connect(this, SIGNAL(showPath(QString, QString)), mainWindow, SLOT(displayAssignedPath(QString, QString)));

    connect(this, SIGNAL(newHome(QString)), mainWindow, SLOT(setNewHome(QString)));

    hide();
    /*setMaximumWidth(mainWindow->width()*4/10);
    setMinimumWidth(mainWindow->width()*4/10);
    inWidget->setMaximumWidth(mainWindow->width()*4/10);
    inWidget->setMinimumWidth(mainWindow->width()*4/10);*/
    inLayout->setAlignment(Qt::AlignTop);
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
    ipAddressLabel->setText("Ip : "+robotView->getRobot()->getIp());
    wifiNameEdit->setText(robotView->getRobot()->getWifi());

    /// If the robot has a home, we display the name of the point, otherwise a default text
    if(robotView->getRobot()->getHome() != NULL){
        homeBtn->setText("Edit Home");
        oldHome = robotView->getRobot()->getHome();
        homeLabel->setText("Home : "+robotView->getRobot()->getHome()->getPoint()->getName());

    } else {
        homeBtn->setText("Assign a home point");
        homeLabel->setText("Home : ");
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

void EditSelectedRobotWidget::setEnableAll(const bool enable){
    qDebug() << "editselectedrobotwidget::setenableall called" << enable;
    nameEdit->setEnabled(enable);
    homeBtn->setEnabled(enable);
    saveBtn->setEnabled(enable);
    deletePathBtn->setEnabled(enable);
    wifiNameEdit->setEnabled(enable);
    wifiPwdEdit->setEnabled(enable);
    nameEdit->setEnabled(enable);
    addPathBtn->setEnabled(enable);
    assignPathButton->setEnabled(enable);
}

void EditSelectedRobotWidget::showEvent(QShowEvent *event){
    setEnableAll(true);
    emit showEditSelectedRobotWidget();
    updatePathsMenu();
    updateHomeMenu();
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
    pathSpaceWidget->hide();
    deletePathBtn->hide();
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
                if(!assignedPath.compare(it_paths.key())){
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
    paths->setVisiblePath(action->text());
    emit showPath(groupName, action->text());
}

void EditSelectedRobotWidget::openHomeMenu(){
    homeMenu->exec(QCursor::pos());
}

void EditSelectedRobotWidget::assignHome(QAction *action){
    action->setCheckable(true);
    action->setChecked(true);
    QString homeName = action->text();
    setHome(points->findPointView(homeName));
    homeLabel->setText("Home : " + homeName);
    homeLabel->wordWrap();
    action->setEnabled(false);
    emit newHome(homeName);
}

void EditSelectedRobotWidget::updateHomeMenu(){
    if(home && oldHome)
        qDebug() << "EditSelectedRobotWidget::updateHomeMenu, home is"
                 << home->getPoint()->getName() << "old home is" << oldHome->getPoint()->getName();
    else
        qDebug() << "EditSelectedRobotWidget::updateHomeMenu, no home";
    homeMenu->clear();
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();
        if(i.value() && i.key().compare(TMP_GROUP_NAME) && i.key().compare(PATH_GROUP_NAME)){
            if(i.value()->count() > 0){
                QMenu *group = homeMenu->addMenu("&" + i.key());
                for(int j = 0; j < i.value()->count(); j++){
                    group->addAction(i.value()->at(j)->getPoint()->getName());
                    /// if a home exists we tick it in the menu
                    if(home && home->getPoint() == i.value()->at(j)->getPoint()){    
                        group->actions().at(j)->setCheckable(true);
                        group->actions().at(j)->setChecked(true);
                    }
                    if(i.value()->at(j)->getPoint()->isHome())
                        group->actions().at(j)->setEnabled(false);
                }
            }
        }
    }
}
