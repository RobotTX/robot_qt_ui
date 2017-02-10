#include "mainwindow.h"
#include <QMap>
#include <QVBoxLayout>
#include <QAbstractButton>
#include <QString>
#include <QStringList>
#include <QVector>
#include <assert.h>
#include <QtGlobal>
#include <QStringList>
#include <fstream>
#include <chrono>
#include <thread>
#include "ui_mainwindow.h"
#include "Helper/helper.h"
#include "Controller/commandcontroller.h"
#include "Controller/lasercontroller.h"
#include "Controller/settingscontroller.h"
#include "Controller/toplayoutcontroller.h"
#include "Controller/mapcontroller.h"
#include "Controller/pathscontroller.h"
#include "Controller/pointscontroller.h"
#include "Controller/robotscontroller.h"
#include "Model/pathpoint.h"
#include "Model/map.h"
#include "Model/robots.h"
#include "Model/robot.h"
#include "Model/pathpoint.h"
#include "View/customqgraphicsview.h"
#include "View/mapview.h"
#include "View/leftmenu.h"
#include "View/pointview.h"
#include "View/leftmenuwidget.h"
#include "View/editselectedrobotwidget.h"
#include "View/bottomlayout.h"
#include "View/pointsleftwidget.h"
#include "View/robotsleftwidget.h"
#include "View/mapleftwidget.h"
#include "View/displayselectedpoint.h"
#include "View/displayselectedgroup.h"
#include "View/pathcreationwidget.h"
#include "View/groupbuttongroup.h"
#include "View/robotbtngroup.h"
#include "View/pathpainter.h"
#include "View/pointbuttongroup.h"
#include "View/customscrollarea.h"
#include "View/toplayoutwidget.h"
#include "View/customlineedit.h"
#include "View/pathpointcreationwidget.h"
#include "View/pathpointlist.h"
#include "View/pathwidget.h"
#include "View/stylesettings.h"
#include "View/displayselectedpointrobots.h"
#include "View/displayselectedpath.h"
#include "View/groupspathswidget.h"
#include "View/displaypathgroup.h"
#include "View/pathbuttongroup.h"
#include "View/custompushbutton.h"
#include "View/groupspathsbuttongroup.h"
#include "View/custompushbutton.h"
#include "View/customrobotdialog.h"
#include "View/customlabel.h"
#include "View/editmapwidget.h"
#include "View/mergemapwidget.h"
#include "View/settingswidget.h"
#include "View/scanmapwidget.h"
#include "View/drawobstacles.h"
#include "View/robotpositionrecovery.h"


MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    QWidget* mainWidget = new QWidget(this);

    QVBoxLayout* mainLayout = new QVBoxLayout(mainWidget);

    /// centers the msgBox on the middle of the screen
    msgBox.move(mapToGlobal(QPoint(QApplication::desktop()->screenGeometry().width()/2,
                                   QApplication::desktop()->screenGeometry().height()/2)));

    /// Create the toolbar
    topLayoutController = new TopLayoutController(this);
    mainLayout->addWidget(topLayoutController->getTopLayout());

    pointsController = new PointsController(this);

    commandController = new CommandController(this);

    connect(this, SIGNAL(stopAllCmd()), commandController, SLOT(stopAllCommand()));

    connect(commandController, SIGNAL(commandDone(QString, bool, QString, QString, QString, QString, bool, int, QStringList)),
            this, SLOT(commandDoneSlot(QString, bool, QString, QString, QString, QString, bool, int, QStringList)));

    robotsController = new RobotsController(this);

    laserController = new LaserController(robotsController->getRobots(), this);

    QHBoxLayout* bottom = new QHBoxLayout();    

    /// need to create the tmp point view so we can create the map but at the same time the map needs to be created before
    /// the points can be initialized that is why we create the map controller first with the map and
    /// the points before the mapView
    mapController = new MapController(robotsController->getRobots(), this);

    pathsController = new PathsController(this, pointsController->getPoints());

    /// the points are only set here because we needed the map to be initialized before
    pointsController->initializePoints();
    pointsController->initializeMenus(this, robotsController->getRobots(), mapController->getMap());

    robotsController->initializeMenus(this);

    /// the temporary point view is only set here because initializePoints() was needed before
    mapController->setTmpPointView(pointsController->getPoints()->getTmpPointView());

    /// button to save the zoom and the position of the map
    connect(topLayoutController->getTopLayout()->getSaveButton(), SIGNAL(clicked()), mapController, SLOT(saveMapState()));


    /// settings of the application (battery level warning threshold, which map to choose between the map of the robot and the map of the app, etc...
    settingsController = new SettingsController(this);

    leftMenu = new LeftMenu(this, pointsController->getPoints(), robotsController->getRobots(), mapController->getMap());

    initializeLeftMenu();
    bottom->addWidget(leftMenu);
    leftMenu->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

    rightLayout = new QVBoxLayout();
    bottom->addLayout(rightLayout);

    rightLayout->addWidget(mapController->getGraphicsView());

    initializeBottomPanel();

    mapController->showGraphicsView();


    ///  ------------------------------------------------------- PATHS CONNECTS ----------------------------------------------------------


    /// to add a path point when we click on the map
    connect(mapController, SIGNAL(pathPointSignal(QString,double,double)), pathsController->getPathCreationWidget(), SLOT(addPathPointSlot(QString, double, double)));

    ///  ------------------------------------------------------- ROBOTS CONNECTS ----------------------------------------------------------

    connect(mapLeftWidget->getSaveBtn(), SIGNAL(clicked()), mapController, SLOT(saveMapState()));

    connect(this, SIGNAL(newBatteryLevel(int)), this, SLOT(updateBatteryLevel(int)));

    mainLayout->addLayout(bottom);

    setCentralWidget(mainWidget);

    /// Centers the map and initialize the map state

    centerMap();

    /// Some style

    setAutoFillBackground(true);

    rightLayout->setContentsMargins(0, 0, 0, 0);
    bottom->setContentsMargins(0, 0, 0, 0);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    bottomLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(0);

    robotsController->launchServer(this);
}

MainWindow::~MainWindow(){
    delete ui;

    if(editMapWidget)
        delete editMapWidget;
}

/**********************************************************************************************************************************/

//                                          ROBOTS

/**********************************************************************************************************************************/


void MainWindow::startScanningSlot(QString robotName){
    qDebug() << "MainWindow::startScanningSlot called" << robotName;

    QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByName(robotName);
    if(robotView){
        if(!commandController->sendCommand(robotView->getRobot(), QString("t"), "", "", "", true))
            emit startedScanning(robotName, false);
    } else
        emit startedScanning(robotName, false);
}

void MainWindow::stopScanningSlot(QStringList listRobot){
    qDebug() << "MainWindow::stopScanningSlot";

    for(int i = 0; i < listRobot.count(); i++){
        QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByName(listRobot.at(i));
        if(robotView && robotView->getRobot()){
            if(!commandController->sendCommand(robotView->getRobot(), QString("u")))
                qDebug() << "MainWindow::stopScanningSlot Could not stop the robot" << listRobot.at(i) << "to scan, stopped trying after 5 attempts";
        } else
            qDebug() << "MainWindow::stopScanningSlot Trying to stop the robot" << listRobot.at(i) << "to scan, but the RobotView could not be found, the robot is probably disconnected";
    }
}

void MainWindow::playScanSlot(bool scan, QString robotName){
    qDebug() << "MainWindow::playScanSlot called" << robotName << scan;

    QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByName(robotName);
    if(robotView){
        if(scan){
            /// If the robot is scanning or was scanning, gmapping is launched so we just want to subscribe to get the map
            /// else we ask if we want to relaunch gmapping and start the scan again
            if(robotView->getRobot()->isScanning()){
                if(!commandController->sendCommand(robotView->getRobot(), QString("e"), "", "", "", scan))
                    emit robotScanning(scan, robotName, false);
            } else {
                QMessageBox msgBox;
                msgBox.setText("The robot restarted, do you wish to restart the scan ?");
                msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
                msgBox.setDefaultButton(QMessageBox::Ok);
                int ret = msgBox.exec();
                switch (ret) {
                    case QMessageBox::Ok:
                        if(!commandController->sendCommand(robotView->getRobot(), QString("t"), "", "", "", false))
                            emit robotScanning(scan, robotName, false);
                    break;
                    case QMessageBox::Cancel:
                        // Cancel was clicked
                        emit robotScanning(scan, robotName, false);
                    break;
                    default:
                        // should never be reached
                    break;
                }
            }
        } else {
            if(!commandController->sendCommand(robotView->getRobot(), QString("f"), "", "", "", scan))
                emit robotScanning(scan, robotName, false);
        }
    } else
        emit robotScanning(scan, robotName, false);
}

void MainWindow::robotGoToSlot(QString robotName, double x, double y){
    qDebug() << "MainWindow::robotGoToSlot" << robotName << "trying to go to" << x << y;
    Position posInRobotCoordinates = Helper::Convert::pixelCoordToRobotCoord(Position(x, y), mapController->getMap()->getOrigin().getX(), mapController->getMap()->getOrigin().getY(), mapController->getMap()->getResolution(), mapController->getMap()->getHeight());
    qDebug() << "MainWindow::robotGoToSlot converted in robot coord to" << posInRobotCoordinates.getX() << posInRobotCoordinates.getY();

    QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByName(robotName);
    if(robotView)
        commandController->sendCommand(robotView->getRobot(), QString("c \"") +
                                       QString::number(posInRobotCoordinates.getX()) + "\" \"" +
                                       QString::number(posInRobotCoordinates.getY()) + "\" \"0\"");
}

void MainWindow::deletePath(int robotNb){
    qDebug() << "MainWindow::deletepath called on robot :" << robotsController->getRobots()->getRobotsVector().at(robotNb)->getRobot()->getName();
    QPointer<Robot> robot = robotsController->getRobots()->getRobotsVector().at(robotNb)->getRobot();
    if(robot->getPath().size() > 0){
        /// if the robot is not playing its path
        if(!robot->isPlayingPath()){
            msgBox.setIcon(QMessageBox::Question);
            int ret = openConfirmMessage("Are you sure you want to delete this path ?");
            switch (ret) {
                case QMessageBox::Ok:
                    /// if the command is succesfully sent to the robot, we apply the change
                    if(!commandController->sendCommand(robot, QString("k"), "", "", "", false, robotNb))
                        topLayoutController->setLabel(TEXT_COLOR_DANGER, "Failed to delete the path of " + robot->getName() + ", please try again");
                break;
                case QMessageBox::Cancel:
                    qDebug() << "Cancel was clicked";
                break;
                default:
                    Q_UNREACHABLE();
                    qDebug() << "Should never be reached";
                break;
            }
        } else {
            msgBox.setIcon(QMessageBox::Warning);
            int ret = openConfirmMessage(robot->getName() + " is currently playing this path, if you delete it the robot will be stopped even if it"
                                                            "has not reached its destination yet. Continue ?");
            switch (ret) {
                case QMessageBox::Ok:
                    /// if the command is succesfully sent to the robot, we apply the change
                    if(!commandController->sendCommand(robot, QString("m"), "", "", "", false, robotNb))
                        topLayoutController->setLabel(TEXT_COLOR_DANGER, "Failed to delete the path of " + robot->getName() + ", please try again");
                break;
                case QMessageBox::Cancel:
                    qDebug() << "Cancel was clicked";
                break;
                default:
                    Q_UNREACHABLE();
                    qDebug() << "Should never be reached";
                break;
            }
        }
    } else
        qDebug() << "This robot has no path";
}

void MainWindow::stopPath(int robotNb){
    qDebug() << "MainWindow::StopPath called";
    QPointer<Robot> robot = robotsController->getRobots()->getRobotsVector().at(robotNb)->getRobot();
    if(!commandController->sendCommand(robot, QString("l"), "", "", "", false))
        topLayoutController->setLabel(TEXT_COLOR_DANGER, "Path failed to be stopped, please try again");
}

void MainWindow::playSelectedRobot(int robotNb){
    QPointer<Robot> robot = robotsController->getRobots()->getRobotsVector().at(robotNb)->getRobot();
    if(robot->isPlayingPath()){
        qDebug() << "MainWindow::playSelectedRobot pause path on robot " << robotNb << " : " << robot->getName();

        if(!commandController->sendCommand(robot, QString("d"), "", "", "", false, robotNb))
            topLayoutController->setLabel(TEXT_COLOR_DANGER, "Path failed to be stopped, please try again");

    } else {
        qDebug() << "MainWindow::playSelectedRobot play path on robot " << robotNb << " : " << robot->getName();

        if(!commandController->sendCommand(robot, QString("j"), "", "", "", false, robotNb))
            topLayoutController->setLabel(TEXT_COLOR_DANGER, "Path failed to start, please try again");
    }
}

void MainWindow::viewPathSelectedRobot(int robotNb, bool checked){
    qDebug() << "MainWindow::viewPathSelectedRobot called" << robotNb << checked;
    if(checked){
        /// in case we were displaying a path from the menu we make sure the eye button as well as the eye icon are unchecked and hidden respectively
       pathsController->getPathGroupDisplayed()->getActionButtons()->getMapButton()->setChecked(false);
        if(pathsController->getPathGroupDisplayed()->getPathButtonGroup()->getButtonGroup()->checkedButton())
            pathsController->getPathGroupDisplayed()->enableButtons(pathsController->getPathGroupDisplayed()->getPathButtonGroup()->getButtonGroup()->checkedButton());
        displayPathOnMap(false);

        QPointer<Robot> robot = robotsController->getRobots()->getRobotsVector().at(robotNb)->getRobot();
        qDebug() << "MainWindow::viewPathSelectedRobot called on robot" << robot->getName();
        bottomLayout->uncheckViewPathSelectedRobot(robotNb);
        pathsController->getPathPainter()->setCurrentPath(robot->getPath(), "");
        bottomLayout->updateRobot(robotNb, robotsController->getRobots()->getRobotsVector().at(robotNb));

    } else {
        if(pointsController->getDisplaySelectedPoint()
                && pointsController->getDisplaySelectedPoint()->isVisible()
                && pointsController->getDisplaySelectedPoint()->getPointView()->getPoint()->isPath()){

            pointsController->getDisplaySelectedPoint()->getPointView()->hide();
            pointsController->getDisplaySelectedPoint()->setPointView(QSharedPointer<PointView>(), "");
            backEvent();
        }
        emit resetPath();
    }
    if(robotsController->getSelectedRobot() && robotsController->getSelectedRobot()->getRobot()->getHome())
        robotsController->getSelectedRobot()->getRobot()->getHome()->setPixmap(PointView::PixmapType::SELECTED);
}

void MainWindow::setSelectedRobot(QPointer<RobotView> robotView){
    qDebug() << "MainWindow::editselectedrobot robotview" << robotView->getRobot()->getName();
    /// resets the home
    robotsController->getEditSelectedRobotWidget()->setHome(robotView->getRobot()->getHome());

    /// same thing for path
    if(!robotView->getRobot()->getPathName().compare(""))
        robotsController->getEditSelectedRobotWidget()->getDeletePathBtn()->hide();

    else {
        robotsController->getEditSelectedRobotWidget()->getDeletePathBtn()->show();
        qDebug() << robotView->getRobot()->getPathName();
    }

    robotsController->getRobots()->setSelected(robotView);

    hideAllWidgets();

    robotsController->setSelectedRobot(robotView);

    robotsController->getEditSelectedRobotWidget()->setAssignedPath(robotsController->getSelectedRobot()->getRobot()->getPathName());
    robotsController->getEditSelectedRobotWidget()->setGroupPath(robotsController->getSelectedRobot()->getRobot()->getGroupPathName());

    /// message to explain the user how to assign a path or a home to his robot
    topLayoutController->setLabel(TEXT_COLOR_NORMAL, "");
    if(robotsController->getSelectedRobot()->getRobot()->getPath().size() == 0){
        topLayoutController->setLabel(TEXT_COLOR_INFO, "You can assign a path to your robot by clicking the button labeled \"Assign a path\"");
        if(!robotsController->getSelectedRobot()->getRobot()->getHome())
            topLayoutController->setLabel(TEXT_COLOR_INFO, topLayoutController->getLabelText() + "\nYou can assign a home to your robot by clicking the button "
                                                                                                 "labeled \"Assign a home point\"");
    } else
        if(!robotsController->getSelectedRobot()->getRobot()->getHome())
            topLayoutController->setLabel(TEXT_COLOR_INFO, topLayoutController->getLabelText() + "You can assign a home to your robot by clicking the button "
                                                                       "labeled \"Assign a home point\"");

    robotsController->getEditSelectedRobotWidget()->setSelectedRobot(robotsController->getSelectedRobot());
    pathsController->getPathPainter()->setPathDeleted(false);

    viewPathSelectedRobot(robotsController->getRobots()->getRobotId(robotView->getRobot()->getName()), true);
    switchFocus(robotsController->getSelectedRobot()->getRobot()->getName(), robotsController->getEditSelectedRobotWidget(), MainWindow::WidgetType::ROBOT);

    /// it was disable by setEnableAll
    leftMenu->getReturnButton()->setEnabled(true);

    emit resetPathCreationWidget();
    pathsController->getPathPainter()->setCurrentPath(robotView->getRobot()->getPath(), "");

    pointsController->showHomeFromRobotName(robotsController->getSelectedRobot()->getRobot()->getName());
    leftMenu->show();
    robotsController->getEditSelectedRobotWidget()->show();

    /// we uncheck the last robot if such robot exists
    if(bottomLayout->getLastCheckedId() != -1){
        bottomLayout->getViewPathRobotBtnGroup()->button(bottomLayout->getLastCheckedId())->setChecked(false);
        bottomLayout->getRobotBtnGroup()->button(bottomLayout->getLastCheckedId())->setChecked(false);
    }

    if(robotsController->getSelectedRobot()->getRobot()->getPath().size() > 0)
        bottomLayout->getViewPathRobotBtnGroup()->button(robotsController->getRobots()->getRobotId(robotView->getRobot()->getName()))->setChecked(true);

    bottomLayout->getRobotBtnGroup()->button(robotsController->getRobots()->getRobotId(robotView->getRobot()->getName()))->setChecked(true);

    /// if the robot has a home we show the go home button otherwise we hide it
    if(!robotView->getRobot()->getHome())
        robotsController->getEditSelectedRobotWidget()->getGoHomeBtn()->hide();
    else {
        robotsController->getEditSelectedRobotWidget()->getGoHomeBtn()->show();
        robotsController->getSelectedRobot()->getRobot()->getHome()->setPixmap(PointView::PixmapType::SELECTED);
    }
}

void MainWindow::robotBtnEvent(void){
    qDebug() << "robotBtnEvent called";
    leftMenuWidget->hide();
    robotsController->getRobotsLeftWidget()->show();
    switchFocus("Robots", robotsController->getRobotsLeftWidget(), MainWindow::WidgetType::ROBOTS);
}

void MainWindow::deletePathSelecRobotBtnEvent(){
    qDebug() << "MainWindow::deletePathSelecRobotBtnEvent called on robot " << robotsController->getSelectedRobot()->getRobot()->getName();
    deletePath(robotsController->getRobots()->getRobotId(robotsController->getSelectedRobot()->getRobot()->getName()));
}

void MainWindow::setSelectedRobotNoParent(QAbstractButton *button){
    qDebug() << "Setselectedrobotnoparent called with id" << bottomLayout->getRobotBtnGroup()->id(button) << ", last id is" << bottomLayout->getLastCheckedId();
    /// displays the robot on the map
    const int robotId = bottomLayout->getRobotBtnGroup()->id(button);
    /// if the button was already checked we uncheck it
    if(bottomLayout->getLastCheckedId() == bottomLayout->getRobotBtnGroup()->id(button)){
        qDebug() << "gotta hide the robot" << button->text();
        /// hides the left menu
        robotsController->getEditSelectedRobotWidget()->hide();
        leftMenu->hide();
        /// enables the robot buttons (otherwise there is a bug that makes the buttons uncheckable for some reason)
        bottomLayout->uncheckRobotNameBtns();
        /// resets the last check Id to -1 which means, no robot was selected before me
        bottomLayout->setLastCheckedId(-1);
        /// to change the pixmap of the robot on the map
        robotsController->getRobots()->deselect();
        /// we hide the path
        bottomLayout->getViewPathRobotBtnGroup()->button(robotId)->setChecked(false);
        robotsController->resetSelectedRobot();
        pointsController->getPoints()->setPixmapAll(PointView::PixmapType::NORMAL);

    } else if(robotId != -1 ){
        qDebug() << "have to display the robot" << button->text();
        resetFocus();
        /// updates the robot menu on the left to fit this particular robot's information
        setSelectedRobot(robotsController->getRobots()->getRobotViewByName(button->text()));

        robotsController->getEditSelectedRobotWidget()->setGroupPath(robotsController->getRobots()->getRobotViewByName(button->text())->getRobot()->getGroupPathName());
        robotsController->getEditSelectedRobotWidget()->setHome(robotsController->getRobots()->getRobotsVector().at(robotId)->getRobot()->getHome());
        robotsController->getEditSelectedRobotWidget()->setAssignedPath(robotsController->getRobots()->getRobotViewByName(button->text())->getRobot()->getPathName());
        /// updates the last checked id to the id of the current button / robot
        bottomLayout->setLastCheckedId(robotId);
        robotsController->getEditSelectedRobotWidget()->show();
        /// show only the home of the selected robot
        pointsController->showHomeFromRobotName(robotsController->getSelectedRobot()->getRobot()->getName());
        if(robotsController->getSelectedRobot()->getRobot()->getHome())
            robotsController->getSelectedRobot()->getRobot()->getHome()->setPixmap(PointView::PixmapType::SELECTED);
    }
}

void MainWindow::setSelectedRobot(QAbstractButton *button){
    Q_UNUSED(button)
    qDebug() << "select a robot in robot group ";

    resetFocus();

    if(robotsController->getRobotsLeftWidget()->getLastCheckedId() != robotsController->getRobotsLeftWidget()->getBtnGroup()->getBtnGroup()->id(button)){
        robotsController->getRobotsLeftWidget()->setLastCheckedId(robotsController->getRobotsLeftWidget()->getBtnGroup()->getBtnGroup()->id(button));
        robotsController->getRobotsLeftWidget()->getActionButtons()->getEditButton()->setEnabled(true);
        robotsController->getRobotsLeftWidget()->getActionButtons()->getMapButton()->setEnabled(true);
        QPointer<RobotView> mySelectedRobot = robotsController->getRobots()->getRobotViewByName(static_cast<CustomPushButton *> (robotsController->getRobotsLeftWidget()->getBtnGroup()->getBtnGroup()->checkedButton())->text());
        robotsController->getEditSelectedRobotWidget()->setGroupPath(mySelectedRobot->getRobot()->getGroupPathName());
        robotsController->getEditSelectedRobotWidget()->setAssignedPath(mySelectedRobot->getRobot()->getPathName());

        const int robotId = robotsController->getRobotsLeftWidget()->getBtnGroup()->getBtnGroup()->id(button);
        robotsController->getRobotsLeftWidget()->getActionButtons()->getMapButton()->setChecked(mySelectedRobot->isVisible());
        /// to show the selected robot with a different color
        robotsController->getRobots()->deselect();
        robotsController->getRobots()->getRobotsVector().at(robotId)->setSelected(true);
        /// to select the robot in the bottom layout accordingly
        bottomLayout->uncheckRobotNameBtns();
        bottomLayout->getRobotBtnGroup()->button(robotId)->setChecked(true);
        bottomLayout->setLastCheckedId(robotId);
    } else {
        robotsController->getRobotsLeftWidget()->getBtnGroup()->uncheck();
        robotsController->getRobotsLeftWidget()->getActionButtons()->getMapButton()->setChecked(false);
        robotsController->getRobotsLeftWidget()->setLastCheckedId(-1);
        robotsController->getRobots()->deselect();
        bottomLayout->uncheckRobotNameBtns();
        bottomLayout->setLastCheckedId(-1);
        robotsController->getRobotsLeftWidget()->getActionButtons()->getEditButton()->setEnabled(false);
        robotsController->getRobotsLeftWidget()->getActionButtons()->getMapButton()->setEnabled(false);
    }
}

void MainWindow::selectViewRobot(){
    qDebug() << "MainWindow::selectViewRobo" <<robotsController->getRobotsLeftWidget()->getSelectedRobotName();
    setSelectedRobot(robotsController->getRobots()->getRobotViewByName(robotsController->getRobotsLeftWidget()->getSelectedRobotName()));
}

void MainWindow::setSelectedRobotFromPointSlot(QString robotName){
    qDebug() << "MainWindow::setSelectedRobotFromPointSlot called :" << robotName;
    QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByName(robotName);
    if(robotView)
        setSelectedRobot(robotView);
    else
        qDebug() << "MainWindow::setSelectedRobotFromPointSlot : Error could not find the robot named :" << robotName;
}

void MainWindow::backRobotBtnEvent(){
    qDebug() << "MainWindow::backRobotBtnEvent called";
    robotsController->getRobotsLeftWidget()->hide();
    leftMenuWidget->show();
}

void MainWindow::editRobotBtnEvent(){
    qDebug() << "MainWindow::editRobotBtnEvent called";
    /// hides a previously shown stand-alone path
    emit resetPath();
    setSelectedRobot(robotsController->getRobots()->getRobotViewByName(static_cast<CustomPushButton*> (robotsController->getRobotsLeftWidget()->getBtnGroup()->getBtnGroup()->checkedButton())->text()));
}

void MainWindow::checkRobotBtnEventMenu(){
    qDebug() << "MainWindow::checkRobotBtnEventMenu called";
    QString name = robotsController->getRobotsLeftWidget()->getBtnGroup()->getBtnGroup()->checkedButton()->text();
    checkRobotBtnEvent(name);
}

void MainWindow::checkRobotBtnEvent(QString name){
    qDebug() << "MainWindow::checkRobotBtnEvent called" << name;
    QPointer<RobotView> robotView =  robotsController->getRobots()->getRobotViewByName(name);
    robotView->display(!robotView->isVisible());
}

void MainWindow::cancelEditSelecRobotBtnEvent(){
    qDebug() << "MainWindow::cancelEditSelecRobotBtnEvent called";
    /// resets the name
    if(robotsController->getEditSelectedRobotWidget()->getHome())
        robotsController->getEditSelectedRobotWidget()->getHomeLabel()->setText("Home : " + robotsController->getEditSelectedRobotWidget()->getHome()->getPoint()->getName());
    else
        robotsController->getEditSelectedRobotWidget()->getHomeLabel()->setText("Home : ");
    /// if a home has been edited we reset it to its old value which might be a null pointer
    if(robotsController->getEditSelectedRobotWidget()->getHome()){
        qDebug() << "MainWindow::cancelEditSelecRobotBtnEvent my home is" << robotsController->getEditSelectedRobotWidget()->getHome()->getPoint()->getName();
        robotsController->getEditSelectedRobotWidget()->getHome()->getPoint()->setHome(Point::PERM);
        robotsController->getEditSelectedRobotWidget()->getHome()->setPixmap(PointView::PixmapType::NORMAL);
    }

    robotsController->getEditSelectedRobotWidget()->setHome((robotsController->getEditSelectedRobotWidget()->getHome()) ? robotsController->getEditSelectedRobotWidget()->getHome() : static_cast<QSharedPointer<PointView>> (0));

    robotsController->getEditSelectedRobotWidget()->updateHomeMenu();
    /// if the path has been changed, reset the path
    emit resetPathCreationWidget();
    displayAssignedPath(robotsController->getSelectedRobot()->getRobot()->getGroupPathName(), robotsController->getSelectedRobot()->getRobot()->getPathName());

    robotsController->getEditSelectedRobotWidget()->setGroupPath(robotsController->getSelectedRobot()->getRobot()->getGroupPathName());
    robotsController->getEditSelectedRobotWidget()->setAssignedPath(robotsController->getSelectedRobot()->getRobot()->getPathName());
    robotsController->getEditSelectedRobotWidget()->setHome(robotsController->getEditSelectedRobotWidget()->getHome());
    robotsController->getEditSelectedRobotWidget()->updatePathsMenu();

    /// the user may have done a mistake regarding which parameter he wants to modify but he still wants to modify something
    /// so it might be better to stay on the same page
    //backEvent();
    robotsController->getEditSelectedRobotWidget()->setPathChanged(false);
    pointsController->getPoints()->getTmpPointView()->setPixmap(PointView::PixmapType::MID);
    pointsController->getPoints()->getTmpPointView()->hide();

    leftMenu->getReturnButton()->setEnabled(true);
    leftMenu->getReturnButton()->setToolTip("");

    setEnableAll(true);

    //setTemporaryMessageTop(TEXT_COLOR_INFO, "You have cancelled all the modifications made to the robot " + robotsController->getSelectedRobot()->getRobot()->getName(), 2500);
}

void MainWindow::saveRobotModifications(){
    qDebug() << "MainWindow::saveRobotModifications called";

    CustomRobotDialog* robotDialog = robotsController->getEditSelectedRobotWidget()->getRobotInfoDialog();
    QString name = robotDialog->getNameEdit()->text();

    QString ssid = robotDialog->getSSIDEdit()->text();
    QString password = robotDialog->getPasswordEdit()->text();

    qDebug() << "MainWindow::saveRobotModifications" << robotsController->getSelectedRobot()->getRobot()->getWifi() << ssid
             << (!name.isEmpty() && name.compare(robotsController->getSelectedRobot()->getRobot()->getName(), Qt::CaseSensitive))
             << (!password.isEmpty() && password.compare("......"))
             << (!ssid.isEmpty() && ssid.compare(robotsController->getSelectedRobot()->getRobot()->getWifi(), Qt::CaseSensitive));

    /// we check if the name & the wifi has been changed
    if((!name.isEmpty() && name.compare(robotsController->getSelectedRobot()->getRobot()->getName(), Qt::CaseSensitive)) &&
            ((!password.isEmpty() && password.compare("......")) ||
            (!ssid.isEmpty() && ssid.compare(robotsController->getSelectedRobot()->getRobot()->getWifi(), Qt::CaseSensitive)))){

        if(commandController->sendCommand(robotsController->getSelectedRobot()->getRobot(), QString("g \"") + name + "\" \"" + ssid + "\" \"" + password + "\"", name)){
            commandDoneNewName(true, name);
            robotsController->getEditSelectedRobotWidget()->getWifiNameLabel()->setText(ssid);
            robotsController->getSelectedRobot()->getRobot()->setWifi(ssid);

            robotDialog->getSSIDEdit()->setText(robotsController->getSelectedRobot()->getRobot()->getWifi());
            robotDialog->getPasswordEdit()->setText("......");
            topLayoutController->setLabel(TEXT_COLOR_SUCCESS, "You have successfully updated" + name);
        } else {
            topLayoutController->setLabel(TEXT_COLOR_DANGER, "Failed to update the robot, please try again");
            return;
        }
    } else {
        /// we check if the name has been changed
        if(!name.isEmpty() && name.compare(robotsController->getSelectedRobot()->getRobot()->getName(), Qt::CaseSensitive)){
            if(!commandController->sendCommand(robotsController->getSelectedRobot()->getRobot(), QString("a \"") + name + "\"", name)){
                topLayoutController->setLabel(TEXT_COLOR_DANGER, "Failed to edit the name of the robot, please try again");
                return;
            }

        /// we check if the SSID or the password have changed
        } else if((!password.isEmpty() && password.compare("......")) ||
                (!ssid.isEmpty() && ssid.compare(robotsController->getSelectedRobot()->getRobot()->getWifi(), Qt::CaseSensitive))){

            if(commandController->sendCommand(robotsController->getSelectedRobot()->getRobot(), QString("b \"") + ssid + "\" \"" + password + "\"")){
                robotsController->getEditSelectedRobotWidget()->getWifiNameLabel()->setText(ssid);
                robotsController->getSelectedRobot()->getRobot()->setWifi(ssid);

                robotDialog->getSSIDEdit()->setText(robotsController->getSelectedRobot()->getRobot()->getWifi());
                robotDialog->getPasswordEdit()->setText("......");
                topLayoutController->setLabel(TEXT_COLOR_SUCCESS, "You have successfully updated" + name);
            } else {
                topLayoutController->setLabel(TEXT_COLOR_DANGER, "Failed to edit the wifi of the robot, please try again");
                return;
            }
        }
    }

    robotsController->getEditSelectedRobotWidget()->getRobotInfoDialog()->close();
}

void MainWindow::editTmpPathPointSlot(int id, QString name, double x, double y){
    qDebug() << "MainWindow::editTmpPathPointSlot called : " << id << name << x << y;
    setGraphicItemsState(GraphicItemState::NO_EVENT);

    leftMenu->setEnableReturnCloseButtons(false);

    topLayoutController->setLabel(TEXT_COLOR_INFO, "Drag the selected point or click the map and click \"Save changes\" to modify the path");
    int nbWidget = pathsController->getPathPainter()->nbUsedPointView(name, x ,y);

    mapController->setMapState(GraphicItemState::EDITING_PATH);

    pointsController->editTmpPathPoint(id, nbWidget);
}

void MainWindow::savePathSlot(){
    qDebug() << "MainWindow::savePath called";
    /// we hide the points that we displayed for the edition of the path
    pointsController->hidePointViewsToDisplayButPath(pathsController->getPathPainter()->getCurrentPath());

    bool already_existed = pathsController->deletePath();

    leftMenu->setEnableReturnCloseButtons(true);

    backEvent();

    (already_existed) ? topLayoutController->setLabel(TEXT_COLOR_SUCCESS, "You have successfully updated the path \"" + pathsController->getPathCreationWidget()->getNameEdit()->text().simplified() + "\" within the group \"" + pathsController->getPathCreationWidget()->getCurrentGroupName() + "\"") :
                        topLayoutController->setLabel(TEXT_COLOR_SUCCESS, "You have successfully created the path \"" + pathsController->getPathCreationWidget()->getNameEdit()->text().simplified() + "\" within the group \"" + pathsController->getPathCreationWidget()->getCurrentGroupName() + "\"");
}

void MainWindow::addPointPathSlot(QString name, double x, double y, GraphicItemState){
    qDebug() << "MainWindow::addPathPoint called on point via * point" << x << y;
    /// Relay to pathPainter::addPathPointSlot()
    emit addPathPoint(name, x, y);
}

void MainWindow::saveEditPathPointSlot(){
    qDebug() << "MainWindow::saveEditPathPointSlot called";

    setEnableAll(false, GraphicItemState::CREATING_PATH, true);

    pathsController->getPathPainter()->updateCurrentPath();

    pointsController->getEditedPointView()->setFlag(QGraphicsItem::ItemIsMovable, false);
    emit updatePathPainter(true);
}

void MainWindow::cancelEditPathPointSlot(){
    qDebug() << "MainWindow::cancelEditPathPointSlot called";

    setEnableAll(false, GraphicItemState::CREATING_PATH, true);

    Position pos;
    int id(-1);

    id = pathsController->getPathCreationWidget()->getPathPointList()->row(pathsController->getPathCreationWidget()->getPathPointList()->currentItem());
    pos = pathsController->getPathPainter()->getCurrentPath().at(id)->getPoint().getPosition();

    pointsController->getEditedPointView()->setPos(pos.getX(), pos.getY());

    emit updatePathPainter(true);

    pointsController->getEditedPointView()->setFlag(QGraphicsItem::ItemIsMovable, false);
    pointsController->resetEditedPointView();
}

void MainWindow::updatePathPainterPointViewSlot(){
    /// Relay to pathPainter::updatePathPainterSlot()
    emit updatePathPainterPointView();
}

void MainWindow::showHome(){
    //qDebug() << "MainWindow::showHome called" << (robotsController->getSelectedRobot()->getRobot()->getHome()==NULL);

    pointsController->getPoints()->setPixmapAll(PointView::PixmapType::NORMAL);

    if(robotsController->getSelectedRobot() && robotsController->getSelectedRobot()->getRobot() && robotsController->getSelectedRobot()->getRobot()->getHome() != NULL){
        QSharedPointer<PointView> pointView = robotsController->getSelectedRobot()->getRobot()->getHome();
        if(pointView->isVisible()){
            qDebug() << "home is visible";
            pointView->setWasShown(true);
        } else {
            qDebug() << "home is not visible";
            pointView->setWasShown(false);
        }

        pointView->show();
    }

    if(!robotsController->getEditSelectedRobotWidget()->isEditing()){
        bottomLayout->uncheckAllViewPath();
        if(pathsController->getPathPainter()->getOldPath().size() > 0){
            robotsController->getEditSelectedRobotWidget()->setPath(pathsController->getPathPainter()->getOldPath());

            pathsController->getPathPainter()->clearOldPath();
        } else {
            QPointer<RobotView> robotView =  robotsController->getRobots()->getRobotViewByName(robotsController->getSelectedRobot()->getRobot()->getName());
            /// If the robot has a path, we display it, otherwise we show the button to add the path
            if(robotView->getRobot()->getPath().size() > 0){
                bottomLayout->getViewPathRobotBtnGroup()->button(robotsController->getRobots()->getRobotId(robotsController->getSelectedRobot()->getRobot()->getName()))->setChecked(true);

                robotsController->getEditSelectedRobotWidget()->getPathWidget()->setPath(robotView->getRobot()->getPath());
                robotsController->getEditSelectedRobotWidget()->getPathWidget()->show();

                robotsController->getEditSelectedRobotWidget()->setPath(robotView->getRobot()->getPath());
            } else {
                qDebug() << "MainWindow::showHome I don't have a path !";
                robotsController->getEditSelectedRobotWidget()->getPathWidget()->hide();

                robotsController->getEditSelectedRobotWidget()->clearPath();
            }
        }
    }
}

void MainWindow::showEditHome(){
    showHome();
    if(!robotsController->getEditSelectedRobotWidget()->isEditing()){
        robotsController->getEditSelectedRobotWidget()->setEditing(true);
    }
}

void MainWindow::clearPath(const int robotNb){
    qDebug() << "MainWindow::clearPath called";

    emit resetPath();
    emit resetPathCreationWidget();
    robotsController->getRobots()->getRobotsVector().at(robotNb)->getRobot()->clearPath();
    bottomLayout->uncheckAllViewPath();
    pathsController->getPathPainter()->setPathDeleted(true);

    /// to uncheck the previously checked path
    robotsController->getRobots()->getRobotsVector().at(robotNb)->getRobot()->setPathName("");
    robotsController->getRobots()->getRobotsVector().at(robotNb)->getRobot()->setGroupPathName("");

    /// serializes the new path (which is actually an empty path)
    QFile fileInfo(QDir::currentPath() + QDir::separator() + "robotsController->getRobots()_paths" + QDir::separator()
                   + robotsController->getRobots()->getRobotsVector().at(robotNb)->getRobot()->getName() + "_path");
    if(fileInfo.open(QIODevice::ReadWrite)){
        fileInfo.resize(0);
        QTextStream out(&fileInfo);
        QString currentDateTime = QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss");
        out << currentDateTime;
        out << "%" << " " << "%" << " ";
        qDebug() << "date now is" << currentDateTime;
        fileInfo.close();
    }

    if(robotsController->getRobots()->getRobotsVector().at(robotNb)->getRobot()->isPlayingPath()){
        qDebug() << "MainWindow::clearPath pause path on robot before supp " << robotNb << " : " << robotsController->getRobots()->getRobotsVector().at(robotNb)->getRobot()->getName();
        robotsController->getRobots()->getRobotsVector().at(robotNb)->getRobot()->setPlayingPath(0);
        bottomLayout->getPlayRobotBtnGroup()->button(robotNb)->setIcon(QIcon(":/icons/play.png"));
    }

    if(robotsController->getEditSelectedRobotWidget()->isVisible()){
        robotsController->getEditSelectedRobotWidget()->setSelectedRobot(robotsController->getRobots()->getRobotsVector().at(robotNb));
        robotsController->getEditSelectedRobotWidget()->setPathChanged(true);
        robotsController->getEditSelectedRobotWidget()->clearPath();
        robotsController->getEditSelectedRobotWidget()->updatePathsMenu();
        robotsController->getEditSelectedRobotWidget()->getPathWidget()->hide();
    }

    bottomLayout->updateRobot(robotNb, robotsController->getRobots()->getRobotsVector().at(robotNb));
}

void MainWindow::showAllHomes(void){
    qDebug() << "MainWindow::showAllHomes called after editselectrobot went hidden";
    /// shows the home of each robot
    robotsController->resetSelectedRobot();
    pathsController->getPathPainter()->setCurrentPath(pathsController->getPathPainter()->getCurrentPath(), pathsController->getPathPainter()->getVisiblePath());
    bottomLayout->uncheckRobotNameBtns();
}

void MainWindow::robotIsAliveSlot(QString hostname, QString ip, QString ssid, int stage, int battery){
    QRegExp rx("[0-9]{1,3}\\.[0-9]{1,3}\\.[0-9]{1,3}\\.[0-9]{1,3}");
    rx.indexIn(ip);
    ip = rx.cap(0);
    QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByIp(ip);

    if(robotView){
        qDebug() << "Robot" << hostname << "at ip" << ip << "is still alive";
        robotView->getRobot()->ping();

    } else {
        qDebug() << "MainWindow::robotIsAliveSlot Robot" << hostname << "at ip" << ip << "just connected";
        QPointer<Robot> robot = QPointer<Robot>(new Robot(this, pathsController->getPaths(), hostname, ip));
        robot->setWifi(ssid);
        robotView = QPointer<RobotView>(new RobotView(robot, mapController->getMapView()));
        connect(robotView, SIGNAL(setSelectedSignal(QPointer<RobotView>)), this, SLOT(setSelectedRobot(QPointer<RobotView>)));
        connect(robotView, SIGNAL(updateLaser()), this, SLOT(updateLaserSlot()));
        robotView->setPosition(robotsController->getRobots()->getRobotsVector().count()*100+100, robotsController->getRobots()->getRobotsVector().count()*100+100);
        robotView->setParentItem(mapController->getMapView());
        robotsController->getRobots()->add(robotView);
        robot->launchWorkers(this);
        bottomLayout->addRobot(robotView);
        robotsController->getRobotsLeftWidget()->updateRobots(robotsController->getRobots());


        /// Check if connection by usb
        if(ip.endsWith(".7.1") || ip.endsWith(".7.2") || ip.endsWith(".7.3")){
            hideAllWidgets();
            robotsController->setSelectedRobot(robotView);
            switchFocus(hostname, robotsController->getEditSelectedRobotWidget(), MainWindow::WidgetType::ROBOT);
            robotsController->getEditSelectedRobotWidget()->setSelectedRobot(robotsController->getSelectedRobot());
            robotsController->getEditSelectedRobotWidget()->show();
            leftMenu->show();
            setEnableAll(false, GraphicItemState::NO_EVENT);
        } else {
            QMap<QString, QString> tmp = robotsController->getRobots()->getRobotsNameMap();
            tmp[ip] = hostname;
            robotsController->getRobots()->setRobotsNameMap(tmp);
            QFile fileWrite(QDir::currentPath() + QDir::separator() + QString(ROBOTS_NAME_FILE));
            fileWrite.resize(0);
            fileWrite.open(QIODevice::WriteOnly);
            QDataStream out(&fileWrite);
            out << robotsController->getRobots()->getRobotsNameMap();
            fileWrite.close();
            qDebug() << "MainWindow::robotIsAliveSlot RobotsNameMap updated" << robotsController->getRobots()->getRobotsNameMap();
        }
    }

    int robotId = robotsController->getRobots()->getRobotId(robotView->getRobot()->getName());

    /// updates the text in the bottom layout to make the stage appear
    if(robotView->getLastStage() != stage){
        robotView->setLastStage(stage);
        bottomLayout->updateStageRobot(robotId, robotView, stage);
        if(stage < 0){
            commandController->sendCommand(robotView->getRobot(), QString("d"), "", "", "", false, robotId);
            QMessageBox::warning(this, "An element is blocking a robot", "An element is blocking the robot " + robotView->getRobot()->getName() + ", please try moving again once the path is cleared.");
        }
    }

    /// if the robot's page is open the progress bar is refreshed to reflect the battery level
    if(robotsController->getSelectedRobot() && robotsController->getSelectedRobot()->getRobot()->getIp() == ip)
        emit newBatteryLevel(battery);

    /// if the battery runs low we send a warning to the user (only when the threshold is just reached so that we don't send
    /// the warning repeatedly
    if(battery < settingsController->getSettings()->getBatteryWarningThreshold() && robotView->getRobot()->getBatteryLevel() == settingsController->getSettings()->getBatteryWarningThreshold()) {
        QMessageBox::warning(this, "Running low on battery", robotView->getRobot()->getName() + " is running low on battery, perhaps you should think about charging it soon");
    }

    robotView->getRobot()->setBatteryLevel(battery);

    /// Check the current stage of the robot
    if(robotView->getRobot()->isPlayingPath() && robotView->getRobot()->getPath().size() == stage){
        topLayoutController->setLabel(TEXT_COLOR_SUCCESS, "The robot " + robotView->getRobot()->getName() + " has successfully reached its destination");
        bottomLayout->getPlayRobotBtnGroup()->button(robotId)->setIcon(QIcon(":/icons/play.png"));
        bottomLayout->getStopRobotBtnGroup()->button(robotId)->setEnabled(false);
    }
}

void MainWindow::robotIsDeadSlot(QString hostname, QString ip){
    qDebug() << "MainWindow::robotIsDeadSlot Robot" << hostname << "at ip" << ip << "... He is dead, Jim!!";
    topLayoutController->setLabel(TEXT_COLOR_DANGER, QString("Robot " + hostname + " at ip " + ip + " disconnected."));

    settingsController->removeRobot(robotsController->getRobots()->getRobotViewByIp(ip)->getRobot()->getName());

    qDebug() << "MainWindow::robotIsDeadSlot Robots IPs : ";
    for(int i = 0; i < robotsController->getRobots()->getRobotsVector().size(); i++){
        qDebug() << robotsController->getRobots()->getRobotsVector().at(i)->getRobot()->getIp();
    }

    QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByIp(ip);
    int id = robotsController->getRobots()->getRobotId(hostname);

    if(robotView && robotView->getRobot() != NULL){

        /// if the robot had a home, make the point a normal point
        if(robotView->getRobot()->getHome() != NULL)
            robotView->getRobot()->getHome()->getPoint()->setHome(Point::PointType::PERM);

        /// if selected => if one of this robot related menu is open
        if(robotsController->getSelectedRobot() && robotsController->getSelectedRobot()->getRobot()->getIp().compare(ip) == 0){
            if(robotsController->getEditSelectedRobotWidget()->isVisible()){
                setGraphicItemsState(GraphicItemState::NO_STATE);
                if(robotsController->getEditSelectedRobotWidget()->getRobotInfoDialog()->isVisible())
                    emit cancelRobotModifications();
                hideAllWidgets();
                leftMenu->hide();
            }


            /// if a box to save/edit this robot is open
            if(msgBox.isVisible())
                msgBox.close();

            robotsController->resetSelectedRobot();
        }

        /// if the robot is scanning send signal to ScanMapWidget
        emit robotDisconnected(hostname);

        /// we stop the robots threads
        robotView->getRobot()->deleteLater();

        /// delete robotview
        mapController->removeFromScene(robotView);

        /// remove from the model
        robotsController->getRobots()->remove(robotView);

        robotView->deleteLater();

        /// update robotsController->getRobotsLeftWidget()
        robotsController->getRobotsLeftWidget()->updateRobots(robotsController->getRobots());

        /// bottomLayout
        bottomLayout->removeRobot(id);

        topLayoutController->removeRobotWithoutHome(hostname);

        mapController->updateMap();

        qDebug() << "MainWindow::robotIsDeadSlot Done removing robot" << hostname << "at ip" << ip;
        topLayoutController->setLabel(TEXT_COLOR_DANGER, QString("Robot " + hostname + " at ip " + ip +" disconnected."));
    } else {
        qDebug() << "MainWindow::robotIsDeadSlot A problem occured, the RobotView or its Robot are NULL, I have been kill twice ?";
    }
}

void MainWindow::setMessageCreationPath(QString message){
    topLayoutController->setLabel(TEXT_COLOR_DANGER, message);
    delay(2500);
    if(robotsController->getSelectedRobot())
        topLayoutController->setLabel(TEXT_COLOR_INFO, "Click white points of the map to add new points to the path of " +
                  robotsController->getSelectedRobot()->getRobot()->getName() + "\nAlternatively you can click the \"+\" button to add an existing point to your path"
                  "\nYou can re-order the points in the list by dragging them");
    else
        topLayoutController->setLabel(TEXT_COLOR_INFO, "Click white points of the map to add new points to your path\n"
                                       "Alternatively you can click the \"+\" button to add an existing point to your path"
                                       "\nYou can re-order the points in the list by dragging them");
}

void MainWindow::updateEditedPathPoint(double x, double y){
    qDebug() << "MainWindow::updateEditedPathPoint called";

    if(pointsController->getEditedPointView())
        pointsController->getEditedPointView()->setPos(x, y);
    else
        qDebug() << "MainWindow::updateEditedPathPoint Could not find the pointView to edit";


    emit updatePathPainter(false);

    if(mapController->getMap()->getMapImage().pixelColor(x, y).red() >= 254){
        topLayoutController->setLabel(TEXT_COLOR_INFO, "You can click either \"Save changes\" to modify your path permanently or \"Cancel\" to keep the original path. If you want you can keep editing your point");
        pathsController->enableSaveEditButton(true);
    } else {
        topLayoutController->setLabel(TEXT_COLOR_DANGER, "You cannot save the current path because the point that you are editing is not in a known area of the map");
        pathsController->enableSaveEditButton(false);
    }
}

void MainWindow::moveEditedPathPointSlot(){
    emit updatePathPainter(false);
    qDebug() << "MainWindow::moveEditedPathPointSlot";
    if(mapController->getPixelColor(pointsController->getEditedPointView()->getPoint()->getPosition().getX(), pointsController->getEditedPointView()->getPoint()->getPosition().getY()).red() >= 254){
        topLayoutController->setLabel(TEXT_COLOR_INFO, "You can click either \"Save changes\" to modify your path permanently or \"Cancel\" to keep the original path. If you want you can keep editing your point");
        pathsController->enableSaveEditButton(true);
    } else {
        topLayoutController->setLabel(TEXT_COLOR_DANGER, "You cannot save the current path because the point that you are editing is not in a known area of the map");
        pathsController->enableSaveEditButton(false);
    }
}

void MainWindow::sendNewMapToRobots(QString ipAddress){
    qDebug() << "sendNewMapToRobots Map id and date :" << mapController->getMap()->getMapId() << mapController->getMap()->getDateTime().toString("yyyy-MM-dd-hh-mm-ss");

    QVector<QPointer<RobotView>> robotsVector = robotsController->getRobots()->getRobotsVector();

    /// We send the map to each robot
    for(int i = 0; i < robotsVector.size(); i++){
        QPointer<Robot> robot = robotsVector.at(i)->getRobot();

        /// No need to send the map to the robot that scanned it
        if(robot->getIp().compare(ipAddress) != 0){
            qDebug() << "Sending the map to" << robot->getName() << "at ip" << robot->getIp();
            robot->sendNewMap(mapController->getMap());
        } else {
            qDebug() << "The robot" << robot->getName() << "at ip" << robot->getIp() << "already has the current map";
        }
    }
    qDebug() << "Sent the map to the robotsController->getRobots()";
}

void MainWindow::updateAllPaths(const Point& old_point, const Point& new_point){
    qDebug() << "MainWindow::updateAllPaths called";
    for(int i = 0; i < robotsController->getRobots()->getRobotsVector().size(); i++){
        QPointer<Robot> robot = robotsController->getRobots()->getRobotsVector().at(i)->getRobot();
        /// to update the description of the path of each robot
        robotsController->getEditSelectedRobotWidget()->setSelectedRobot(robotsController->getRobots()->getRobotViewByName(robot->getName()));
        QVector<QSharedPointer<PathPoint>> path = robot->getPath();
        for(int j = 0; j < path.size(); j++){
            Point point = path.at(j)->getPoint();
            /// if the point is the old point we make the necessary changes
            if(point.comparePos(old_point.getPosition())){
                qDebug() << "New point position: " << new_point.getPosition().getX() << new_point.getPosition().getY();
                if(!point.comparePos(new_point.getPosition())){
                    qDebug() << "Will replace name by path point name + number ";
                    point.setName(PATH_POINT_NAME + QString::number(j+1));
                }
                else {
                    qDebug() << new_point.getName() << " name has been edited";
                    point.setName(new_point.getName());
                }
                path.at(j)->setPoint(point);
            }
        }
        robot->setPath(path);
        /// updates the list of path points (in case it was containing a permanent point which has been modified)
        robotsController->getEditSelectedRobotWidget()->setAssignedPath(robotsController->getEditSelectedRobotWidget()->getAssignedPath());
        bottomLayout->updateRobot(robotsController->getRobots()->getRobotId(robot->getName()), robotsController->getRobots()->getRobotsVector().at(i));
    }

    if(robotsController->getEditSelectedRobotWidget()->getRobot()){
        /// to reset the tooltip of the edited point !
        int robotId = robotsController->getRobots()->getRobotId(robotsController->getEditSelectedRobotWidget()->getRobot()->getRobot()->getName());
        if(bottomLayout->getViewPathRobotBtnGroup()->button(robotId)->isChecked()){
            qDebug() << " i am displayed " << robotId;
            viewPathSelectedRobot(robotId, false);
            viewPathSelectedRobot(robotId, true);
        }
    }

    /// updates the paths of the model
    pathsController->updatePaths(old_point, new_point);

    if(pathsController->getVisiblePath().compare("")){
        /// to set the tooltip of the modified point
        for(int i = 0; i < pathsController->getPathPainter()->getCurrentPath().size(); i++){
            Point currPoint = pathsController->getPathPainter()->getCurrentPath().at(i)->getPoint();
            if(currPoint.comparePos(old_point.getPosition())){
                if(currPoint.comparePos(new_point.getPosition()))
                    currPoint.setName(new_point.getName());
            }
        }
        pathsController->getPathPainter()->setCurrentPath(pathsController->getPathPainter()->getCurrentPath(), pathsController->getPathPainter()->getVisiblePath());
    }
}

void MainWindow::resetPathPointViewsSlot(){
    emit updatePathPainter(false);
}

void MainWindow::setNewHome(QString homeName){

    /// retrieves the pointview which name has been clicked in the menu
    QSharedPointer<PointView> home = pointsController->getPoints()->findPointView(homeName);

    if(home->getPoint()->setHome(Point::PointType::HOME)){

        Position posInRobotCoordinates = Helper::Convert::pixelCoordToRobotCoord(home->getPoint()->getPosition(), mapController->getMap()->getOrigin().getX(), mapController->getMap()->getOrigin().getY(), mapController->getMap()->getResolution(), mapController->getMap()->getHeight());
        if(!commandController->sendCommand(robotsController->getSelectedRobot()->getRobot(), QString("n \"") + QString::number(posInRobotCoordinates.getX()) + "\" \""
                                                          + QString::number(posInRobotCoordinates.getY()) + "\"", homeName, "", "", false, 0))
            topLayoutController->setLabel(TEXT_COLOR_DANGER, robotsController->getSelectedRobot()->getRobot()->getName() + " failed to save its home point, please try again");
    } else
        topLayoutController->setLabel(TEXT_COLOR_DANGER, "Sorry, this point is already a home\nPlease select another");

}

void MainWindow::goHome(){
    qDebug() << "MainWindow::goHome called (soon soon working)";
    if(!commandController->sendCommand(robotsController->getSelectedRobot()->getRobot(), QString("o")))
        topLayoutController->setLabel(TEXT_COLOR_DANGER, "Failed to send the robot " + robotsController->getSelectedRobot()->getRobot()->getName() + " home, please try again");
}

void MainWindow::goHome(int nbRobot){
    qDebug() <<"MainWindow::GoHome (bottomlayout) called";
    QPointer<Robot> currRobot = robotsController->getRobots()->getRobotsVector().at(nbRobot)->getRobot();
    if(!currRobot->isPlayingPath()){
        if(!commandController->sendCommand(currRobot, QString("o")))
            topLayoutController->setLabel(TEXT_COLOR_DANGER, "Failed to send the robot " + currRobot->getName() + " home, please try again");
    } else {
        int answer = openConfirmMessage("The robot " + currRobot->getName() + " is currently playing its path. Do you want to stop it and send it home anyway ?");
        switch(answer){
            case QMessageBox::Cancel:
            break;
            case QMessageBox::Ok:
                if(!commandController->sendCommand(currRobot, QString("o")))
                    topLayoutController->setLabel(TEXT_COLOR_DANGER, "Failed to send the robot " + currRobot->getName() + " home, please try again");
            break;
            default:
            break;
        }
    }
}

void MainWindow::updateBatteryLevel(const int level){
    if(robotsController->getEditSelectedRobotWidget())
        robotsController->getEditSelectedRobotWidget()->setBatteryLevel(level);
}

/**
 * @brief MainWindow::doubleClickOnRobot
 * @param id
 * does the same as clicking on a robot and then on the eye button
 */
void MainWindow::doubleClickOnRobot(QString id){
    qDebug() << "double click on robot" << id;
    setSelectedRobot(robotsController->getRobots()->getRobotViewByName(id));
}

void MainWindow::openPositionRecoveryWidget() {
    if(!robotPositionRecoveryWidget) {
        robotPositionRecoveryWidget = QPointer<RobotPositionRecovery>(new RobotPositionRecovery(robotsController->getRobots()));
        openHelpMessage("You are about to recover the position of one or more of your robots. This is how to proceed...\n\n\t"
                        "* Choose a robot and click on \"start recovering the position\""
                        "* You can also click the map to set your own goals"
                        "* When the position has been recovered this window will automatically close", "recover_robot_position");

    } else
        robotPositionRecoveryWidget->activateWindow();
}

/**********************************************************************************************************************************/

//                                          MAPS

/**********************************************************************************************************************************/

void MainWindow::updateMetadata(const int width, const int height, const float resolution,
                                const float originX, const float originY){

    mapController->updateMetadata(width, height, resolution, originX, originY);
}

void MainWindow::mapReceivedSlot(const QByteArray mapArray, int who, QString mapId, QString mapDate, QString resolution, QString originX, QString originY, QString ipAddress){
    qDebug() << "MainWindow::mapReceivedSlot received a map" << who;

    if(who == 2){
        qDebug() << "MainWindow::mapReceivedSlot received a map from a robot to merge" << ipAddress << resolution << originX << originY;
        QString robotName = robotsController->getRobots()->getRobotViewByIp(ipAddress)->getRobot()->getName();
        QImage image = mapController->getImageFromArray(mapArray, true);

        emit receivedMapToMerge(robotName, image, resolution.toDouble(), originX.toDouble(), originY.toDouble());

    } else if(who == 1){
        mapController->modifyMap(mapArray, who, mapId, mapDate);
        mapController->updateScene();

    } else {
        qDebug() << "MainWindow::mapReceivedSlot received a map while scanning";
        QString robotName = robotsController->getRobots()->getRobotViewByIp(ipAddress)->getRobot()->getName();
        QImage image = mapController->getImageFromArray(mapArray, false);

        emit receivedScanMap(robotName, image, resolution.toDouble());
    }
}

void MainWindow::saveMapBtnEvent(){
    qDebug() << "saveMapBtnEvent called";

    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
            "", tr("Images (*.pgm)"));

    saveMap(fileName);
}

void MainWindow::saveMap(QString fileName){
    if(!fileName.isEmpty()){
        if(fileName.indexOf(".pgm", fileName.length()-4) != -1)
            fileName = fileName.mid(0, fileName.length()-4);

        mapController->saveMapState();

        topLayoutController->setLabel(TEXT_COLOR_INFO, "The current configuration of the map has been saved");

        QFileInfo mapFileInfo(static_cast<QDir> (fileName), "");
        QFileInfo fileInfo(QDir::currentPath(), "../gobot-software/mapConfigs/" + mapFileInfo.fileName() + ".config");
        qDebug() << fileInfo.absoluteFilePath();

        mapController->updateMapFile(fileName.toStdString() + ".pgm");

        mapController->saveMapConfig((QDir::currentPath() + QDir::separator() + "currentMap.txt").toStdString());

        assert(mapController->saveMapConfig(fileInfo.absoluteFilePath().toStdString()));

        /// saves the new configuration to the map configuration file
        const QString pointsFile = fileName + "_points.xml";
        pointsController->savePoints(pointsFile);

        /// saves the new configuration to the current configuration file
        pointsController->savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

        /// saves the map
        mapController->saveMapToFile(fileName + ".pgm");

        const QString pathsFile = fileName + "_paths.dat";

        /// saves the current configuration for the paths (this configuration will be associated to the map
        /// when you load the map in the future
        pathsController->serializePaths(pathsFile);

        for(int i = 0; i < robotsController->getRobots()->getRobotsVector().size(); i++)
            robotsController->getRobots()->getRobotsVector().at(i)->getRobot()->sendNewMap(mapController->getMap());

    }
}

void MainWindow::loadMapBtnEvent(){
    qDebug() << "loadMapBtnEvent called";
    QMessageBox box;
    box.setText("Warning, loading a new map will erase all previously created points, paths and selected home of robotsController->getRobots(). Do you wish to save your current configuration first ?");
    QAbstractButton* saveButton = box.addButton("Save and load", QMessageBox::YesRole);
    box.addButton("Load", QMessageBox::YesRole);
    QPushButton* cancelButton = box.addButton("Cancel", QMessageBox::NoRole);
    box.setDefaultButton(cancelButton);

    box.exec();

    if(box.clickedButton() == cancelButton) return;

    if(box.clickedButton() == saveButton)
        saveMapBtnEvent();

    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Image"), "", tr("Image Files (*.pgm)"));

    if(!fileName.isEmpty()){
        QString fileNameWithoutExtension;
        if(fileName.indexOf(".pgm", fileName.length()-4) != -1)
            fileNameWithoutExtension = fileName.mid(0, fileName.length()-4);

        QFileInfo mapFileInfo(static_cast<QDir> (fileNameWithoutExtension), "");
        QFileInfo fileInfo(QDir::currentPath(), "../gobot-software/mapConfigs/" + mapFileInfo.fileName() + ".config");
        qDebug() << fileInfo.absoluteFilePath() << "map to load";
        /// if we are able to find the configuration then we load the map
        if(mapController->loadMapConfig(fileInfo.absoluteFilePath().toStdString())){

            for(int i = 0; i < robotsController->getRobots()->getRobotsVector().size(); i++)
                robotsController->getRobots()->getRobotsVector().at(i)->getRobot()->sendNewMap(mapController->getMap());

            /// clears the map of all paths and points
            clearNewMap();

            mapController->saveMapState();

            mapController->modifyMap();

            mapController->updateScene();

            /// centers the map
            centerMap();

            /// imports paths associated to the map and save them in the current file
            pathsController->deserializePaths(fileNameWithoutExtension + "_paths.dat");

            /// imports points associated to the map and save them in the current file
            pointsController->loadPoints(fileNameWithoutExtension + "_points.xml");

            /// savesthe new configuration to the current configuration file
            pointsController->savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

            /// updates the group box so that new points can be added
            pointsController->getCreatePointWidget()->updateGroupBox();

            /// saves the imported paths in the current paths file
            pathsController->serializePaths(QDir::currentPath() + QDir::separator() + "paths.dat");

            /// updates the groups of paths menu using the paths that have just been imported
            pathsController->updateGroupsPaths();

        } else {
            QMessageBox warningBox;
            warningBox.setText("No configuration found for this map.");
            warningBox.setStandardButtons(QMessageBox::Ok);
            warningBox.setDefaultButton(QMessageBox::Ok);
            warningBox.exec();
        }
    }
}

void MainWindow::mapBtnEvent(){
    qDebug() << "MainWindow::mapBtnEvent called";
    leftMenuWidget->hide();
    mapLeftWidget->show();
    switchFocus("Menu", mapLeftWidget, MainWindow::WidgetType::MAP);
}

void MainWindow::messageMapSaved(bool status){
    (status) ? setTemporaryMessageTop(TEXT_COLOR_SUCCESS, "You have successfully saved the map", 2500) :
               setTemporaryMessageTop(TEXT_COLOR_DANGER, "Attempt to save the map failed", 2500);
}

void MainWindow::editMapSlot(){
    editMapWidget = QPointer<EditMapWidget>(new EditMapWidget(mapController->getMapImage(), mapController->getMapWidth(), mapController->getMapHeight(), mapController->getMapResolution(), mapController->getMapOrigin()));
    openHelpMessage("You are about to edit a map, here is how to proceed...\n\n\t"
                    "* Select a color\n\n\t"
                    "* Select a a shape\n\n\t"
                    "* Select a size\n\n\t"
                    "* Undo (Ctrl+Z) or redo (Ctrl+Y) "
                    "your actions using the arrows\n\n\t"
                    "* Click reset to start from scratch\n\n\t"
                    "* Don't forget to either cancel or save your modifications\n",
                    "edit_map");
    connect(editMapWidget, SIGNAL(saveEditMap()), this, SLOT(saveEditMapSlot()));
}

void MainWindow::saveEditMapSlot(){
    qDebug() << "MainWindow::saveEditMapSlot called";
    if(editMapWidget){
        QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
                "", tr("Images (*.pgm)"));

        /// If the user press cancel on the box, the filename fill be empty
        if(!fileName.isEmpty()){
            qDebug() << "MainWindow::saveEditMapSlot called with filename" << fileName;

            mapController->updateMap();

            mapController->updateScene();

            saveMap(fileName);
            editMapWidget->close();

            sendNewMapToRobots();
        }
    } else {
        qDebug() << "MainWindow::saveEditMapSlot called while editMapWidget is not set, duh ?";
    }
}

void MainWindow::mergeMapSlot(){
    qDebug() << "MainWindow::mergeMapSlot called";
    topLayoutController->setLabel(TEXT_COLOR_INFO, "You can select a map by clicking it or by clicking the list in the menu."
                                         "\nYou can move a map by dragging and dropping it or by using the directional keys."
                                         "\nYou can rotate the map in the menu using the text block or the slider.");

    mergeMapWidget = QPointer<MergeMapWidget>(new MergeMapWidget(robotsController->getRobots()));
    openHelpMessage("You are about to merge two or more maps together, here is how to proceed...\n\n\t"
                    "* Import two or more maps from the robot or from any folder\n\n\t"
                    "* Drag and rotate your maps until you are satisfied\n\n\t"
                    "* Click reset to start from scratch\n\n\t"
                    "* Don't forget to save or cancel your modifications\n", "merge_maps");

    connect(mergeMapWidget, SIGNAL(saveMergeMap(double, Position, QImage, QString)), this, SLOT(saveMergeMapSlot(double, Position, QImage, QString)));
    connect(mergeMapWidget, SIGNAL(getMapForMerging(QString)), this, SLOT(getMapForMergingSlot(QString)));
    connect(this, SIGNAL(receivedMapToMerge(QString, QImage, double, double, double)), mergeMapWidget, SLOT(receivedMapToMergeSlot(QString, QImage, double, double, double)));
}

void MainWindow::saveMergeMapSlot(double resolution, Position origin, QImage image, QString fileName){
    qDebug() << "MainWindow::saveMergeMapSlot called with filename" << fileName;

    clearNewMap();

    mapController->updateMap(fileName.toStdString(), resolution, image.width(), image.height(), origin, image, QUuid::createUuid(), QDateTime::currentDateTime());

    mapController->updateScene();

    saveMap(fileName);

    sendNewMapToRobots();
    topLayoutController->setLabel(TEXT_COLOR_SUCCESS, "The new merged map has been save successfully");
}

void MainWindow::saveScanMapSlot(double resolution, Position origin, QImage image, QString fileName){
    qDebug() << "MainWindow::saveScanMapSlot called with filename" << fileName;

    clearNewMap();

    mapController->updateMap(fileName.toStdString(), resolution, image.width(), image.height(), origin, image, QUuid::createUuid(), QDateTime::currentDateTime());

    mapController->updateScene();

    saveMap(fileName);

    sendNewMapToRobots();
    topLayoutController->setLabel(TEXT_COLOR_SUCCESS, "The new scanned map has been save successfully");
}

void MainWindow::teleopCmdSlot(QString robotName, int id){
    qDebug() << "MainWindow::teleopCmdSlot called" << robotName << id;
    QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByName(robotName);
    if(robotView && robotView->getRobot())
        robotView->getRobot()->sendTeleopCmd(id);
}

void MainWindow::scanMapSlot(){
    qDebug() << "MainWindow::scanMapSlot called";

    if(!scanMapWidget){
        scanMapWidget = QPointer<ScanMapWidget>(new ScanMapWidget(robotsController->getRobots()));
        openHelpMessage("You are about to scan the map. This is how to proceed...\n\n\t"
                        "* Use the teleop keys to make the robot\n\n\t"
                        "* Click the map to set goals from the robot", "scan");

        connect(scanMapWidget, SIGNAL(startScanning(QString)), this, SLOT(startScanningSlot(QString)));
        connect(scanMapWidget, SIGNAL(stopScanning(QStringList)), this, SLOT(stopScanningSlot(QStringList)));
        connect(scanMapWidget, SIGNAL(playScan(bool, QString)), this, SLOT(playScanSlot(bool, QString)));
        connect(scanMapWidget, SIGNAL(robotGoTo(QString, double, double)), this, SLOT(robotGoToSlot(QString, double, double)));
        connect(scanMapWidget, SIGNAL(saveScanMap(double, Position, QImage, QString)), this, SLOT(saveScanMapSlot(double, Position, QImage, QString)));
        connect(scanMapWidget, SIGNAL(teleopCmd(QString, int)), this, SLOT(teleopCmdSlot(QString, int)));

        connect(this, SIGNAL(startedScanning(QString, bool)), scanMapWidget, SLOT(startedScanningSlot(QString, bool)));
        connect(this, SIGNAL(robotDisconnected(QString)), scanMapWidget, SLOT(robotDisconnectedSlot(QString)));
        connect(this, SIGNAL(robotReconnected(QString)), scanMapWidget, SLOT(robotReconnectedSlot(QString)));
        connect(this, SIGNAL(robotScanning(bool,QString,bool)), scanMapWidget, SLOT(robotScanningSlot(bool,QString,bool)));
        connect(this, SIGNAL(receivedScanMap(QString,QImage,double)), scanMapWidget, SLOT(receivedScanMapSlot(QString,QImage,double)));
        connect(robotsController, SIGNAL(scanRobotPos(QString, double, double, double)), scanMapWidget, SLOT(scanRobotPosSlot(QString, double, double, double)));

    } else
        scanMapWidget->activateWindow();
}

void MainWindow::updateLaserSlot(){
    mapController->getObstaclesPainter()->update();
}

/**********************************************************************************************************************************/

//                                          MENUS

/**********************************************************************************************************************************/


void MainWindow::initializeLeftMenu(){
    lastWidgets =  QList<QPair<QPair<QWidget*,QString>, MainWindow::WidgetType>>();
    leftMenuWidget = leftMenu->getLeftMenuWidget();
    mapLeftWidget = leftMenu->getMapLeftWidget();
}

void MainWindow::initializeBottomPanel(){
    bottomLayout = new BottomLayout(this, robotsController->getRobots());
    rightLayout->addWidget(bottomLayout);
}

void MainWindow::closeSlot(){
    resetFocus();
    pointsController->getDisplaySelectedPoint()->hide();
    leftMenu->hide();
    setEnableAll(true);
    if(pointsController->getDisplaySelectedPoint()->getPointView())
        pointsController->getDisplaySelectedPoint()->getPointView()->setPixmap(PointView::PixmapType::NORMAL);
}

/**********************************************************************************************************************************/

//                                          POINTS

/**********************************************************************************************************************************/


/**
 * @brief MainWindow::setSelectedTmpPoint
 * @param pointView
 * set the temporary point as the selected point
 */
void MainWindow::setSelectedTmpPoint(){
    qDebug() << "MainWindow::setSelectedTmpPoint called";

    resetFocus();

    int id = bottomLayout->getViewPathRobotBtnGroup()->checkedId();
    if(id > 0)
        pathsController->getPathPainter()->setCurrentPath(robotsController->getRobots()->getRobotsVector().at(id)->getRobot()->getPath(), "");

    leftMenu->show();

    hideAllWidgets();

    pointsController->setSelectedTmpPoint(this);

    switchFocus(TMP_POINT_NAME, pointsController->getCreatePointWidget(), MainWindow::WidgetType::POINT);

    /// to deselect a potentially selected robot
    bottomLayout->uncheckRobotNameBtns();
    robotsController->getRobots()->deselect();
    bottomLayout->setLastCheckedId(-1);
}

/**
 * @brief MainWindow::pointBtnEvent
 * called when the back button is clicked
 */
void MainWindow::pointBtnEvent(void){
    /// resets the list of groups menu
    switchFocus("Groups", pointsController->getPointsLeftWidget(), MainWindow::WidgetType::GROUPS);
    qDebug() << "pointBtnEvent called ";
    /// we uncheck all buttons from all menus
    pointsController->getDisplaySelectedGroup()->uncheck();
    hideAllWidgets();
    pointsController->getPointsLeftWidget()->show();
    topLayoutController->setLabel(TEXT_COLOR_INFO, "Click the map to add a permanent point");
}

/**
 * @brief MainWindow::pointSavedEvent
 * @param index
 * @param x
 * @param y
 * @param name
 * called when a permanent point is created from a temporary one using enter or the save button
 */
void MainWindow::pointSavedEvent(QString groupName, double x, double y, QString name){

    qDebug() << "MainWindow::pointSavedEvent called" << groupName;
    leftMenu->getCloseButton()->setEnabled(true);
    leftMenu->getReturnButton()->setEnabled(true);

    pointsController->pointSavedEvent(groupName, x, y, name);

    /// hide the creation widget
    hideAllWidgets();
    leftMenu->hide();

    topLayoutController->setLabel(TEXT_COLOR_SUCCESS, "You have successfully added the new point \"" + name + "\" to the group : \"" + groupName + "\"");
}


void MainWindow::enableReturnAndCloseButtons(){
    leftMenu->getReturnButton()->setEnabled(true);
    leftMenu->getCloseButton()->setEnabled(true);
    topLayoutController->enableLayout(true);
}

void MainWindow::setMessageCreationPoint(QString type, CreatePointWidget::Error error){
    qDebug() << "MainWindow::setMessageCreation point called from mainwindow";
    switch(error){
    case CreatePointWidget::Error::NoError:
        topLayoutController->setLabel(type, "Click save or press ENTER to save this point");
        break;
    case CreatePointWidget::Error::ContainsSemicolon:
        topLayoutController->setLabel(type, "You cannot create a point with a name that contains a semicolon, a curly bracket or the pattern \"pathpoint\"");
        break;
    case CreatePointWidget::Error::EmptyName:
        topLayoutController->setLabel(type, "You cannot create a point with an empty name");
        break;
    case CreatePointWidget::Error::AlreadyExists:
        topLayoutController->setLabel(type, "You cannot create a point with this name because a point with the same name already exists");
        break;
    default:
        Q_UNREACHABLE();
        qDebug() << "Should never be here, if you do get here however, check that you have not added a new error code and forgotten to add it in the cases afterwards";
        break;
    }
}


/**********************************************************************************************************************************/

//                                          PATHS

/**********************************************************************************************************************************/




void MainWindow::pathBtnEvent(){
    hideAllWidgets();
    robotsController->getRobots()->deselect();
    bottomLayout->uncheckRobotNameBtns();
    bottomLayout->setLastCheckedId(-1);
    /// resets the list of groups menu
    switchFocus("Paths", pathsController->getGroupsPathsWidget(), MainWindow::WidgetType::GROUPS_PATHS);
    pathsController->showGroupsPathsWidget();
}

void MainWindow::deletePathSlot(QString groupName, QString pathName){
    qDebug() << "MainWindow::deletePathSlot called on group :" << groupName << ", path :" << pathName;
    int answer = openConfirmMessage("Are you sure you want to delete this path, this action is irreversible ?");
    switch(answer){
    case QMessageBox::StandardButton::Ok:
    {
        pathsController->deletePath(groupName, pathName);
        pathsController->serializePaths(QDir::currentPath() + QDir::separator() + "paths.dat");
        /// we show the other paths of the same group
        pathsController->setPathsGroup(groupName);
        pathsController->showPathsGroup();
        pathsController->hideDisplayedPathWidget();
        if(!pathsController->getVisiblePath().compare(pathName)){
            qDebug() << "hey i have to stop displaying this path that was destroyed";
            emit resetPath();
        }
        backEvent();
        topLayoutController->setLabel(TEXT_COLOR_SUCCESS, "You have successfully deleted the path \"" + pathName + "\" which belonged to the group \"" + groupName + "\"");
    }
    break;
    case QMessageBox::StandardButton::Cancel:
        /// resets the paths group
        pathsController->getPathGroupDisplayed()->getPathButtonGroup()->uncheck();
        pathsController->getPathGroupDisplayed()->setLastCheckedButton("");
    break;
    default:
        Q_UNREACHABLE();
        qDebug() << "MainWindow::deletePath you should not be here, you probably forgot to implement the behavior for one of your buttons";
    break;
    }
}

void MainWindow::editPathSlot(QString groupName, QString pathName){
    qDebug() << "MainWindow::editPathSlot called on group :" << groupName << ", path :" << pathName;
    topLayoutController->setLabel(TEXT_COLOR_INFO, "Click white points of the map to add new points to the path of "
                 "\nAlternatively you can click the \"+\" button to add an existing point to your path"
                 "\nYou can re-order the points in the list by dragging them");

    hideAllWidgets();

    pathsController->editPath(groupName, pathName);

    switchFocus(pathName, pathsController->getPathCreationWidget(), MainWindow::WidgetType::PATH);

    setEnableAll(false, GraphicItemState::CREATING_PATH, true);

    /// hides the temporary pointview
    pointsController->getPoints()->getTmpPointView()->hide();

    /// displays the points in order to make them available for the edition of the path,
    ///  we have to keep track of those which were hidden so we can hide them again once the edition is finished
    pointsController->showPointViewsToDisplay();

    leftMenu->getReturnButton()->setDisabled(true);
}

void MainWindow::displayGroupPaths(){
    /// TODO if can bedone inside controller
    qDebug() << "MainWindow::displayGroupPaths called";
    switchFocus(pathsController->getGroupPathsChecked(),
                pathsController->getPathGroupDisplayed(), MainWindow::WidgetType::GROUP_OF_PATHS);

    pathsController->displayGroupPaths();
}

void MainWindow::createGroupPaths(){
    qDebug() << "MainWindow::createGroupPaths called";
    topLayoutController->setLabel(TEXT_COLOR_INFO, "The name of your group cannot be empty");
    pathsController->prepareGroupPathsCreation();
}

void MainWindow::deleteGroupPaths(){
    qDebug() << "MainWindow::deleteGroupPaths called";
    QString groupPaths(pathsController->getGroupPathsChecked());
    QString message("This group of paths contains the following paths which are assigned to one or more or your robotsController->getRobots() : ");
    QList<int> robotsIds;
    /// to form a proper sentence
    bool first(true);
    /// we check for each robot whether or not one path of the group has been assigned to it
    for(int i = 0; i < robotsController->getRobots()->getRobotsVector().size(); i++){
        QString currentRobotPath = robotsController->getRobots()->getRobotsVector().at(i)->getRobot()->getPathName();
        QMapIterator<QString, QSharedPointer<Paths::Path> > it_paths(pathsController->getPaths()->getGroup(groupPaths));
        while(it_paths.hasNext()){
            it_paths.next();
            if(!it_paths.key().compare(currentRobotPath)){
                if(first){
                    first = false;
                    message += currentRobotPath;
                } else
                    message += ", " + currentRobotPath;

                robotsIds.push_back(i);
            }
        }
    }
    qDebug() << "message" << message;
    message += ". If you delete this group, these robotsController->getRobots() will lose their paths. If you wish to continue click \"Ok\". ";

    /// if none of the paths of this group has been assigned to a robot
    int answer = openConfirmMessage((first) ? "Are you sure you want to delete this group of path, all the paths inside will be deleted as well ?" :
                                          message);

    pathsController->resetGroupsPathsWidget();
    switch(answer){
    case QMessageBox::StandardButton::Ok:
    {
        /// to delete the paths of the robots whose paths are contained in the group which is about to be deleted
        while(!robotsIds.empty()){
            clearPath(robotsIds.front());
            robotsIds.pop_front();
        }
        pathsController->deleteGroup(groupPaths);
        pathsController->serializePaths(QDir::currentPath() + QDir::separator() + "paths.dat");
        pathsController->updateGroupsPaths();
        /// if the displayed path was among the paths of this group, we hide it as we delete it
        emit resetPath();
        topLayoutController->setLabel(TEXT_COLOR_SUCCESS, "You have successfully deleted the group of paths \"" + groupPaths + "\"");
        delay(4000);
        topLayoutController->setLabel(TEXT_COLOR_NORMAL, "");
        break;
    }
    case QMessageBox::StandardButton::Cancel:
        /// unchecks the group button (prettier imo but not necessary on the logical level)
        pathsController->getGroupsPathsWidget()->uncheck();
        break;
    default:
        Q_UNREACHABLE();
        qDebug() << "MainWindow::deleteGroupPaths ended up in the default case which suggests that you forgot to implement the behavior relative to a particular button";
        break;
    }
}

void MainWindow::saveGroupPaths(QString name){
    qDebug() << "saveGroupPaths called" << name;

    name = name.simplified();
    if(pathsController->checkGroupPathName(name) == 0){
        pathsController->getGroupsPathsWidget()->setLastCheckedButton("");

        /// updates the model
        pathsController->createGroup(name);

        /// updates list of groups in menu
        pathsController->updateGroupsPaths();

        /// enables the return button again
        leftMenu->getReturnButton()->setEnabled(true);

        /// hides everything that's related to the creation of a group
        pathsController->hideGroupCreationWidgets();

        /// enables the plus button again
        pathsController->enableGroupsPathsWidgetPlusButtonOnly();

        topLayoutController->enableLayout(true);

        pathsController->serializePaths(QDir::currentPath() + QDir::separator() + "paths.dat");

        topLayoutController->setLabelDelay(TEXT_COLOR_SUCCESS, "You have created a new group of paths", 4000);

    } else if(pathsController->checkGroupPathName(name) == 1){
        /// enables the return button again
        leftMenu->getReturnButton()->setEnabled(true);

        /// hides everything that's related to the creation of a group
        pathsController->hideGroupCreationWidgets();

        /// enables the plus button again
        pathsController->enableGroupsPathsWidgetPlusButtonOnly();

        topLayoutController->enableLayout(true);
        /// TODO check if next line needed
        //leftMenu->getGroupsPathsWidget()->getActionButtons()->getPlusButton()->setEnabled(true);
    }
    else
        topLayoutController->setLabelDelay(TEXT_COLOR_DANGER, "You cannot choose : " + name + " as a new name for your group because another group already has this name", 4000);
}

void MainWindow::modifyGroupPathsWithEnter(QString name){
    name = name.simplified();
    qDebug() << "modifying group paths after enter key pressed from" << pointsController->getPointsLeftWidget()->getLastCheckedId() << "to" << name;

    leftMenu->setEnableReturnCloseButtons(true);

    bool has_changed = pathsController->modifyGroupPathsWithEnter(name);

    /// if the name has really changed
    (has_changed) ? topLayoutController->setLabelDelay(TEXT_COLOR_SUCCESS, "You have successfully modified the name of your group", 4000) : topLayoutController->setLabel(TEXT_COLOR_NORMAL, "");
}

void MainWindow::doubleClickOnPathsGroup(QString checkedButton){
    switchFocus(checkedButton, pathsController->getPathGroupDisplayed(), MainWindow::WidgetType::GROUP_OF_PATHS);
    pathsController->doubleClickOnPathsGroup(checkedButton);
}

void MainWindow::displayPath(){
    const QString groupName = lastWidgets.at(lastWidgets.size()-1).first.second;
    switchFocus(pathsController->getPathGroupDisplayed()->getLastCheckedButton(), pathsController->getDisplaySelectedPath(), MainWindow::WidgetType::PATH);
    pathsController->displayPath(groupName);
}

void MainWindow::createPath(){
    qDebug() << "MainWindow::createPath called";
    /// to prevent a path to be saved with an empty name
    pathsController->getPathCreationWidget()->getNameEdit()->setText("");
    pathsController->getPathCreationWidget()->getSaveButton()->setEnabled(false);

    switchFocus("newPath", pathsController->getPathCreationWidget(), MainWindow::WidgetType::PATH);

    /// to clear the map of any path
    emit resetPath();

    topLayoutController->setLabel(TEXT_COLOR_INFO, "The name of your path cannot be empty, fill up the corresponding field to give your path a name");
    hideAllWidgets();
    setEnableAll(false, GraphicItemState::CREATING_PATH, true);
    pathsController->getPathCreationWidget()->show();

    /// stop displaying the currently displayed path if it exists
    pathsController->getPathCreationWidget()->getPathPointList()->clear();

    bottomLayout->uncheckAllViewPath();

    /// hides the temporary pointview
    pointsController->getPoints()->getTmpPointView()->hide();

    /// displays the points in order to make them available for the edition of the path,
    ///  we have to keep track of those which were hidden so we can hide them again once the edition is finished
    pointsController->showPointViewsToDisplay();
}

void MainWindow::deletePath(){
    qDebug() << "MainWindow::deletePath called";
    deletePathSlot(lastWidgets.at(lastWidgets.size()-1).first.second, pathsController->getPathGroupDisplayed()->getLastCheckedButton());
}

void MainWindow::displayPathOnMap(const bool display){
    /// TODO check if can be done inside controller
    qDebug() << "MainWindow::displayPathOnMap called";
    /// to hide the path drawn by the robot path painter
    pathsController->getPathCreationWidget()->resetPath();
    if(display){
        bottomLayout->uncheckViewPathSelectedRobot(bottomLayout->getLastCheckedId());
        bool foundFlag = false;
        pathsController->getPathPainter()->setCurrentPath(pathsController->getPaths()->getPath(lastWidgets.at(lastWidgets.size()-1).first.second, pathsController->getPathGroupDisplayed()->getLastCheckedButton(), foundFlag), pathsController->getPathGroupDisplayed()->getLastCheckedButton());
    } else {
        pathsController->setVisiblePath("");
        qDebug() << "no path visible !";
    }
    /// to set the 'eye' icon appropriately
    pathsController->updateDisplayedPath();
}

void MainWindow::editPath(){
    qDebug() << "MainWindow::editPath called";
    topLayoutController->setLabel(TEXT_COLOR_INFO, "Click white points of the map to add new points to the path of "
                 "\nAlternatively you can click the \"+\" button to add an existing point to your path"
                 "\nYou can re-order the points in the list by dragging them");

    hideAllWidgets();

    const QString pathName = pathsController->editPath();

    switchFocus(pathName, pathsController->getPathCreationWidget(), MainWindow::WidgetType::PATH);

    bottomLayout->uncheckAllViewPath();

    setEnableAll(false, GraphicItemState::CREATING_PATH, true);

    /// hides the temporary pointview
    pointsController->getPoints()->getTmpPointView()->hide();

    /// displays the points in order to make them available for the edition of the path,
    ///  we have to keep track of those which were hidden so we can hide them again once the edition is finished
    pointsController->showPointViewsToDisplay();

    leftMenu->getReturnButton()->setEnabled(false);
}

void MainWindow::doubleClickOnPath(QString pathName){
    /// TODO check if can be done inside controller
    QString groupName = lastWidgets.at(lastWidgets.size()-1).first.second;
    qDebug() << "MainWindow::displayPath called" << groupName << pathName;
    switchFocus(pathName, pathsController->getDisplaySelectedPath(), MainWindow::WidgetType::PATH);
    pathsController->doubleClickOnPath(pathName, groupName);

}

void MainWindow::sendPathSelectedRobotSlot(const QString groupName, const QString pathName){

    qDebug() << "MainWindow::sendPathSelectedRobotSlot path changed";
    QPointer<Robot> robot = robotsController->getSelectedRobot()->getRobot();
    /// prepares the cmd to send to the robot
    qDebug() << "MainWindow::sendPathSelectedRobotSlot" << pathsController->getPathPainter()->getCurrentPath().size();
    bool found;
    Paths::Path currPath = pathsController->getPath(groupName, pathName, found);
    QString pathStr = prepareCommandPath(currPath);

    /// if the command is succesfully sent to the robot, we apply the change
    if(!commandController->sendCommand(robot, QString("i ") + pathStr, "", groupName, pathName)){
        topLayoutController->setLabel(TEXT_COLOR_DANGER, "The path of " + robot->getName() + "\" could not be updated, please try again");
        qDebug() << "MainWindow::sendPathSelectedRobotSlot Path failed to be saved, please try again";
    }
}

void MainWindow::setMessageNoRobotPath(const int code){
    switch(code){
    case 0:
        topLayoutController->setLabel(TEXT_COLOR_INFO, "You cannot save your path because its name is still empty");
        pathsController->enablePathCreationSaveButton(false);
    break;
    case 1:
        topLayoutController->setLabel(TEXT_COLOR_INFO, "You cannot save your path because the name you chose is already taken by another path in the same group");
        pathsController->enablePathCreationSaveButton(false);
    break;
    case 2:
        topLayoutController->setLabel(TEXT_COLOR_INFO, "You can save your path any time you want by clicking the \"Save\" button");
        pathsController->enablePathCreationSaveButton(true);
    break;
    default:
        Q_UNREACHABLE();
        qDebug() << "MainWindow::setMessageNoRobotPath you should not be here you probably forgot to implement the behavior for the error code" << code;
    break;
    }
}

void MainWindow::cancelNoRobotPathSlot(){
    qDebug() << "MainWindow::cancelNoRobotPathSlot called";

    pathsController->setPathsGroup(pathsController->getPathCreationWidget()->getCurrentGroupName());

    QVector<QSharedPointer<PathPoint>> oldPath = pathsController->getPathPainter()->getOldPath();
    /// we hide the points that we displayed just for the edition of the path
    pointsController->hidePointViewsToDisplay();

    emit resetPathCreationWidget();

    pathsController->getPathCreationWidget()->updatePath(oldPath);

    backEvent();

    pathsController->getPathPainter()->setOldPath(oldPath);
    setEnableAll(false, GraphicItemState::NO_EVENT);
    leftMenu->setEnableReturnCloseButtons(true);
    topLayoutController->enableLayout(true);
    bottomLayout->setEnable(true);

    setTemporaryMessageTop(TEXT_COLOR_INFO, "You have cancelled the modifications of the path \"" + pathsController->getPathCreationWidget()->getCurrentPathName() + "\"", 2500);
}

void MainWindow::saveNoRobotPathSlot(){
    qDebug() << "MainWindow::saveNoRobotPath called";
    backEvent();

    /// gotta update the model and serialize the paths
    const QString groupName = pathsController->getPathCreationWidget()->getCurrentGroupName();
    const QString pathName = pathsController->getPathCreationWidget()->getNameEdit()->text().simplified();
    qDebug() << groupName << pathName;
    pathsController->createPath(groupName, pathName);
    for(int i = 0; i < pathsController->getPathPainter()->getCurrentPath().size(); i++)
        pathsController->addPathPoint(groupName, pathName, pathsController->getPathPainter()->getCurrentPath().at(i));

    /// we hide the points that we displayed for the edition of the path
    pointsController->hidePointViewsToDisplayButPath(pathsController->getPathPainter()->getCurrentPath());

    /// TODO check if param needed
    pathsController->getPathPainter()->setPathDeleted(false);
    pathsController->getPathPainter()->setOldPath(pathsController->getPathPainter()->getCurrentPath());

    setEnableAll(false, GraphicItemState::NO_EVENT);

    leftMenu->setEnableReturnCloseButtons(true);

    /// updates the visible path
    pathsController->setVisiblePath(pathName);

    /// resets the menu so that it reflects the creation of this new path
    pathsController->setPathsGroup(pathsController->getPathCreationWidget()->getCurrentGroupName());

    /// add this path to the file
    pathsController->serializePaths(QDir::currentPath() + QDir::separator() + "paths.dat");

}

void MainWindow::setMessageModifGroupPaths(int code){
    switch(code){
    case 0:
        topLayoutController->setLabel(TEXT_COLOR_INFO, "Press enter to save this name for your group");
        break;
    case 1:
        topLayoutController->setLabel(TEXT_COLOR_INFO, "You cannot have an empty name for your group");
        break;

    case 2:
        topLayoutController->setLabel(TEXT_COLOR_INFO, "You cannot save this name for your group as it is already the name of another group");
        break;
    default:
        Q_UNREACHABLE();
        qDebug() << "MainWindow::setMessageModifGroupPaths You should not be here you probably forgot to implement the behavior for the code" << code;
    }
}

void MainWindow::displayAssignedPath(QString groupName, QString pathName){

    bool foundFlag(true);
    /// TODO check if param needed
    pathsController->getPathPainter()->setCurrentPath(pathsController->getPath(groupName, pathName, foundFlag), pathName);
    robotsController->getEditSelectedRobotWidget()->updatePathsMenu();
    assert(foundFlag);

    robotsController->getSelectedRobot()->getRobot()->setPath(pathsController->getCurrentPathFromPathPainter());
    robotsController->getSelectedRobot()->getRobot()->setGroupPathName(robotsController->getEditSelectedRobotWidget()->getGroupPathName());
    robotsController->getSelectedRobot()->getRobot()->setPathName(robotsController->getEditSelectedRobotWidget()->getPathName());
    int id = robotsController->getRobots()->getRobotId(robotsController->getSelectedRobot()->getRobot()->getName());
    bottomLayout->updateRobot(id, robotsController->getSelectedRobot());
    if(pathsController->getPathPainter()->getCurrentPath().size() > 0){
        bottomLayout->getViewPathRobotBtnGroup()->button(id)->setChecked(true);
        viewPathSelectedRobot(id, true);
    }

    /// we update the path on the application side by serializing the path
    QFile fileInfo(QDir::currentPath() + QDir::separator() + "robotsController->getRobots()_paths" + QDir::separator() + robotsController->getSelectedRobot()->getRobot()->getName() + "_path");
    if(fileInfo.open(QIODevice::ReadWrite)){
        fileInfo.resize(0);
        QTextStream out(&fileInfo);
        QString currentDateTime = QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss");
        out << currentDateTime;
        out << "%" << groupName << "%" << pathName;
        qDebug() << "date now is" << currentDateTime;
        fileInfo.close();
    }
}

void MainWindow::clearMapOfPaths(){
    emit resetPath();
}

/**********************************************************************************************************************************/

//                                          ODDS AND ENDS

/**********************************************************************************************************************************/

void MainWindow::quit(){
    close();
}

void MainWindow::backEvent(){
    setEnableAll(true);
    /// resets the menus
    pointsController->getPointsLeftWidget()->disableButtons();
    pointsController->getPointsLeftWidget()->getGroupButtonGroup()->uncheck();
    pointsController->getDisplaySelectedGroup()->disableButtons();
    pointsController->getDisplaySelectedGroup()->uncheck();

    if(lastWidgets.size() > 0)
        lastWidgets.last().first.first->hide();

    if (lastWidgets.size() > 1){
        lastWidgets.removeLast();
        if(lastWidgets.last().second == MainWindow::WidgetType::GROUP || lastWidgets.last().second == MainWindow::WidgetType::GROUPS)
            topLayoutController->setLabel(TEXT_COLOR_INFO, "Click the map to add a permanent point");

        lastWidgets.last().first.first->show();

        (lastWidgets.size() > 1) ? leftMenu->showBackButton(lastWidgets.at(lastWidgets.size()-2).first.second) :
                                   leftMenu->hideBackButton();

    } else {
        resetFocus();
        leftMenu->hide();
    }
}

/**
 * @brief MainWindow::openConfirmMessage
 * @param text
 * @return int
 * prompts the user for confirmation
 */
int MainWindow::openConfirmMessage(const QString text){
    msgBox.setText(text);
    msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::Cancel);
    return msgBox.exec();
}

/**
 * @brief MainWindow::setGraphicItemsState
 * @param state
 * @param clear
 * resets the state of the map, robotViews et pointViews
 */
void MainWindow::setGraphicItemsState(const GraphicItemState state){
    qDebug() << "MainWindow::setGraphicItemsState called" << state << robotsController->getRobots()->getRobotsVector().size();
    mapController->setMapState(state);

    for(int i = 0; i < robotsController->getRobots()->getRobotsVector().size(); i++)
        robotsController->getRobots()->getRobotsVector().at(i)->setState(state);

    pointsController->getPoints()->setPointViewsState(state);
}

void MainWindow::hideAllWidgets(){
    leftMenuWidget->hide();
    pointsController->getPointsLeftWidget()->hide();
    robotsController->getRobotsLeftWidget()->hide();
    mapLeftWidget->hide();
    robotsController->getEditSelectedRobotWidget()->hide();
    pointsController->getCreatePointWidget()->hide();
    pointsController->getDisplaySelectedPoint()->hide();
    pointsController->getDisplaySelectedGroup()->hide();
    pathsController->hideDisplayedPathWidget();
    pathsController->hidePathCreationWidget();
    pathsController->hideGroupsPathsWidget();
    pathsController->hidePathGroupWidget();
}

void MainWindow::clearNewMap(){
    qDebug() << "clearNewMap called";

    pointsController->resetEditedPointView();

    pathsController->setVisiblePath("");

    emit resetPath();

    /// Clear the list of points
    pointsController->getPoints()->clear();

    /// clears the list of paths
    pathsController->clearPaths();

    /// Update the left menu displaying the list of groups and buttons
    pointsController->getPointsLeftWidget()->updateGroupButtonGroup();

    for(int i = 0; i < robotsController->getRobots()->getRobotsVector().size(); i++){
        robotsController->getRobots()->getRobotsVector().at(i)->getRobot()->clearPath();
        if(robotsController->getRobots()->getRobotsVector().at(i)->getRobot()->getHome()){
            robotsController->getRobots()->getRobotsVector().at(i)->getRobot()->getHome()->hide();
            robotsController->getRobots()->getRobotsVector().at(i)->getRobot()->setHome(QSharedPointer<PointView>());
        }
    }
}

void MainWindow::delay(const int ms){
    QTime dieTime= QTime::currentTime().addMSecs(ms);
    while (QTime::currentTime() < dieTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

void MainWindow::setEnableAll(bool enable, GraphicItemState state, int noReturn){
    setGraphicItemsState(state);
    bottomLayout->setEnable(enable);
    topLayoutController->enableLayout(enable);
    leftMenu->setEnableReturnCloseButtons((noReturn == -1) ? enable : noReturn);
}

void MainWindow::centerMap(){
    qDebug() << " mainwindow center map called" << mapController->getMapState().first << mapController->getMapState().second;
    /// centers the map on the point chosen by the user and not on the center of the map
    mapController->moveMap(mapController->getMapState().first);
    if(mapController->getZoomCoeff() != mapController->getMapState().second)
        mapController->setZoomCoeff(mapController->getMapState().second);
}

void MainWindow::settingBtnSlot(){
    qDebug() << "MainWindow::settingBtnSlot called";
    settingsController->showView();
}

void MainWindow::setTemporaryMessageTop(const QString type, const QString message, const int ms){
    topLayoutController->setLabel(type, message);
    delay(ms);
    topLayoutController->setLabel(TEXT_COLOR_NORMAL, "");
}

void MainWindow::updateRobotInfo(QString robotName, QString robotInfo){

    QStringList strList = robotInfo.split(" ", QString::SkipEmptyParts);
    qDebug() << "MainWindow::updateRobotInfo" << robotInfo << "to" << strList;
    QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByName(robotName);

    if(strList.size() > 7){
        /// Remove the "Connected"
        strList.removeFirst();
        QString mapId = strList.takeFirst();
        QString mapDate = strList.takeFirst();
        QString home_x = strList.takeFirst();
        QString home_y = strList.takeFirst();
        QString homeDate = strList.takeFirst();
        QString pathDate = strList.takeFirst();
        bool scanning = static_cast<QString>(strList.takeFirst()).toInt();
        /// What remains in the list is the path

        updatePathInfo(robotName, pathDate, strList);

        updateHomeInfo(robotName, home_x, home_y, homeDate);

        updateMapInfo(robotName, mapId, mapDate);


        if(robotView && robotView->getRobot())
            robotView->getRobot()->setScanning(scanning);
        else
            return;

        if(scanning){
            if(scanMapWidget){
                emit robotReconnected(robotName);
                playScanSlot(true, robotName);
            } else
                stopScanningSlot(QStringList(robotName));
        } else {
            if(scanMapWidget){
                QStringList robotScanningList = scanMapWidget->getAllScanningRobots();
                for(int i = 0; i < robotScanningList.count(); i++){
                    if(static_cast<QString>(robotScanningList.at(i)) == robotName){
                        emit robotReconnected(robotName);
                        emit robotScanning(false, robotName, true);
                    }
                }
            }
        }
    } else
        qDebug() << "MainWindow::updateRobotInfo Connected received without enough parameters :" << strList;

    settingsController->addRobot(robotName);

}

void MainWindow::updateMapInfo(const QString robotName, QString mapId, QString mapDate){

    /// Check if the robot has the current map
    //qDebug() << "Robot" << robotName << "comparing ids" << mapId << "and" << map->getMapId().toString();
    if(mapId.compare(mapController->getMapId().toString()) == 0){
        qDebug() << "Robot" << robotName << "has the current map";
    } else {
        QDateTime mapDateTime = QDateTime::fromString(mapDate, "yyyy-MM-dd-hh-mm-ss");
        //qDebug() << "Robot" << robotName << "comparing date" << mapDateTime << "and" << map->getDateTime();

        bool robotOlder = mapDateTime <= mapController->getMapTime();
        if(robotOlder){
            qDebug() << "Robot" << robotName << "has a different and older map";
        } else {
            qDebug() << "Robot" << robotName << "has a different and newer map";
        }

        QPointer<Robot> robot = robotsController->getRobots()->getRobotViewByName(robotName)->getRobot();

        switch(settingsController->getSettings()->getSettingMapChoice()){
            case SettingsWidget::ALWAYS_NEW:
               if(robotOlder)
                   robot->sendNewMap(mapController->getMap());
               else
                   commandController->sendCommand(robot, QString("s \"1\""));
            break;
            case SettingsWidget::ALWAYS_OLD:
                if(robotOlder)
                    commandController->sendCommand(robot, QString("s \"1\""));
                else
                    robot->sendNewMap(mapController->getMap());
            break;
            case SettingsWidget::ALWAYS_ROBOT:
                commandController->sendCommand(robot, QString("s \"1\""));
            break;
            case SettingsWidget::ALWAYS_APPLICATION:
                robot->sendNewMap(mapController->getMap());
            break;
            default:
                QMessageBox msgBox;
                QPushButton* robotButton;
                QPushButton* appButton;

                (robotOlder) ? msgBox.setText("The robot " + robotName + " has a new map.") : msgBox.setText("The robot " + robotName + " has an old map.");

                msgBox.setInformativeText("Which map do you want to use ?");
                robotButton = msgBox.addButton(tr("Robot"), QMessageBox::AcceptRole);
                appButton = msgBox.addButton(tr("Application"), QMessageBox::RejectRole);

                msgBox.exec();

                if (msgBox.clickedButton() == robotButton) {
                    qDebug() << "Robot" << robotName << "using the map from the robot";
                    commandController->sendCommand(robot, QString("s \"1\""));
                } else if (msgBox.clickedButton() == appButton) {
                    qDebug() << "Robot" << robotName << "using the map from the app";
                    robot->sendNewMap(mapController->getMap());
                }
                delete robotButton;
                robotButton = 0;
                delete appButton;
                appButton = 0;
            break;
        }
    }
}

bool MainWindow::isLater(const QStringList& date, const QStringList& otherDate){
    assert(date.size() == otherDate.size());
    for(int i = 0; i < date.size(); i++){
        if(date.at(i).toInt() > otherDate.at(i).toInt())
            return true;
        else if(date.at(i).toInt() < otherDate.at(i).toInt())
            return false;
    }
    return false;
}

QPair<Position, QStringList> MainWindow::getHomeFromFile(const QString robotName){
    /// retrieves the home point of the robot if the robot has one
    QFile fileInfo(QDir::currentPath() + QDir::separator() + "robotsController->getRobots()_homes" + QDir::separator() + robotName);
    Position p;
    QStringList dateLastModification;
    if(fileInfo.open(QIODevice::ReadWrite)){
        QRegExp regex("[-\n ]");
        QString content = fileInfo.readAll();
        if(!content.compare(""))
            content = "0-0-1970-01-01-00-00-00";
        content.replace("\n", " ");
        QStringList l = content.split(regex, QString::SkipEmptyParts);
        qDebug() << "app list" << l;
        if(l.size() > 0){
            p.setX(l.at(0).toDouble());
            p.setY(l.at(1).toDouble());
            for(int i = 2; i < l.size(); i++)
                dateLastModification.push_back(l.at(i));
        }
    }
    fileInfo.close();
    return QPair<Position, QStringList> (p, dateLastModification);
}

void MainWindow::setHomeAtConnection(const QString robotName, const Position &pos_home){
    /// the robot and the application have the same point so we set this one as the home of the robot
    QSharedPointer<PointView> home = pointsController->getPoints()->findPointViewByPos(pos_home);
    QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByName(robotName);
    /// Remove the previous home
    if(robotView->getRobot()->getHome()){
        robotView->getRobot()->getHome()->getPoint()->setHome(Point::PERM);
        robotView->getRobot()->getHome()->setPixmap(PointView::PixmapType::NORMAL);
    }

    /// associates the robot to the point
    home->getPoint()->setRobotName(robotView->getRobot()->getName());
    robotView->getRobot()->setHome(home);
    home->getPoint()->setType(Point::HOME);
    robotsController->getEditSelectedRobotWidget()->setSelectedRobot(robotView);

    /// setCurrentPath is displaying the path so if it was not displayed we hide it
    if(!bottomLayout->getViewPathRobotBtnGroup()->button(robotsController->getRobots()->getRobotId(robotView->getRobot()->getName()))->isChecked())
        emit resetPath();

    pointsController->savePoints(QDir::currentPath() + QDir::separator() + "points.xml");
}

bool MainWindow::updateHomeFile(const QString robotName, const Position& robot_home_position, const QStringList date){
    qDebug() << "updatehomefile" << robotName << date.size();
    QFile fileWriteHome(QDir::currentPath() + QDir::separator() + "robotsController->getRobots()_homes" + QDir::separator() + robotName);
    if(fileWriteHome.open(QIODevice::ReadWrite)){
        QTextStream out(&fileWriteHome);
        out << robot_home_position.getX() << " " << robot_home_position.getY() << "\n";
        for(int i = 0; i < date.size()-1; i++)
            out << date.at(i) << "-";
        out << date.at(date.size()-1);
        fileWriteHome.close();
        return true;
    } else {
        qDebug() << "could not update the home of" << robotName;
        return false;
    }
}

QVector<PathPoint> MainWindow::extractPathFromInfo(const QStringList &robotInfo){
    QVector<PathPoint> path;
    for(int i = 0; i < robotInfo.size(); i += 3){
        Position in_app = Helper::Convert::robotCoordToPixelCoord(Position(robotInfo.at(i).toDouble(), robotInfo.at(i+1).toDouble()),
                                                                    mapController->getMapOrigin().getX(), mapController->getMapOrigin().getY(), mapController->getMapResolution(), mapController->getMapHeight());
        path.push_back(PathPoint(Point(" ", in_app.getX(),in_app.getY()), robotInfo.at(i+2).toDouble()));
    }
    return path;
}

void MainWindow::updateHomeInfo(const QString robotName, QString posX, QString posY, QString homeDate){
    QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByName(robotName);

    /// retrieves the home point of the robot if the robot has one
    QPair<Position, QStringList> appHome = getHomeFromFile(robotName);
    Position pos = appHome.first;
    QStringList dateLastModification = appHome.second;

    /// we gotta convert the coordinates first
    Position robot_home_position = Helper::Convert::robotCoordToPixelCoord(Position(posX.toDouble(), posY.toDouble()), mapController->getMapOrigin().getX(), mapController->getMapOrigin().getY(), mapController->getMapResolution(), mapController->getMapHeight());
    QStringList dateHomeOnRobot = homeDate.split("-");

    /// if the robot and the application have the same home we don't do anything besides setting the point in the application (no need to change any files)
    if(robot_home_position != pos){
         qDebug() << "HEY HOMES ARE DIFFERENT" << robot_home_position.getX() << robot_home_position.getY()
                  << pos.getX() << pos.getY();
         QSharedPointer<PointView> home = pointsController->getPoints()->findPointViewByPos(pos);

         if(home){

            /// the application has a home, we need to compare with the robot's home if one exists
            QSharedPointer<PointView> home_sent_by_robot = pointsController->getPoints()->findPointViewByPos(robot_home_position);
            if(home_sent_by_robot){
                qDebug() << "HOME ROBOT" << home->getPoint()->getName();
                /// if the robot's file is more recent we update on the application side and we look for the pointview corresponding to
                /// the coordinates given by the robot
                if(isLater(dateHomeOnRobot, dateLastModification)){
                    setHomeAtConnection(robotName, robot_home_position);
                    updateHomeFile(robotName, robot_home_position, dateHomeOnRobot);
                } else {
                    /// the application has the most recent file, we send the updated coordinates to the robot
                    Position posInRobotCoordinates = Helper::Convert::pixelCoordToRobotCoord(home->getPoint()->getPosition(), mapController->getMapOrigin().getX(), mapController->getMapOrigin().getY(), mapController->getMapResolution(), mapController->getMapHeight());
                    commandController->sendCommand(robotView->getRobot(), QString("n \"") + QString::number(posInRobotCoordinates.getX()) + "\" \""
                                                                      + QString::number(posInRobotCoordinates.getY()) + "\"", "", "", "", false, 2);
                }
            } else {
                qDebug() << "HOME APP" << home->getPoint()->getName();
                Position posInRobotCoordinates = Helper::Convert::pixelCoordToRobotCoord(home->getPoint()->getPosition(), mapController->getMapOrigin().getX(), mapController->getMapOrigin().getY(), mapController->getMapResolution(), mapController->getMapHeight());
                commandController->sendCommand(robotView->getRobot(), QString("n \"") + QString::number(posInRobotCoordinates.getX()) + "\" \""
                                                  + QString::number(posInRobotCoordinates.getY()) + "\"", "", "", "", false, 2);
            }

        } else {
            qDebug() << " the robot is sending its home and the application does not have one";
            /// if the application does not have a home stored on its side but the robot is sending one
            /// that we are able to find
            QSharedPointer<PointView> home_sent_by_robot = pointsController->getPoints()->findPointViewByPos(robot_home_position);
            if(home_sent_by_robot){
                qDebug() << "HOME ROBOT" << home_sent_by_robot->getPoint()->getName();
                /// sets the home inside the application (house icon, extra buttons etc...)
                setHomeAtConnection(robotName, robot_home_position);

                /// updates the home file on the application side
                updateHomeFile(robotName, robot_home_position, dateHomeOnRobot);

            } else {
                robotHasNoHome(robotView->getRobot()->getName());
            }
        }
    } else {
        /// the robot and the application have the same point so we set this one as the home of the robot
        /// no need to modify any files
        QSharedPointer<PointView> home_app = pointsController->getPoints()->findPointViewByPos(pos);
        if(home_app){
            setHomeAtConnection(robotName, pos);
            qDebug() << "HOME APP same on both side" << home_app->getPoint()->getName();
        } else {
            robotHasNoHome(robotView->getRobot()->getName());
        }
    }
    robotsController->getEditSelectedRobotWidget()->setSelectedRobot(robotsController->getRobots()->getRobotViewByName(robotName));
    robotsController->getEditSelectedRobotWidget()->updateHomeMenu();
}

void MainWindow::robotHasNoHome(QString robotName){
    QMessageBox chooseHomeMessageBox;
    chooseHomeMessageBox.setStandardButtons(QMessageBox::Ok);
    chooseHomeMessageBox.setText("\"" + robotName + "\" does not have a home point yet.\n Please choose a home for the robot in the robot menu.");
    chooseHomeMessageBox.exec();
    qDebug() << "HOME ROBOT NO HOME AT ALL";
    topLayoutController->addRobotWithoutHome(robotName);
}

void MainWindow::updatePathInfo(const QString robotName, QString pathDate, QStringList path){
    QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByName(robotName);

    /// retrieves the path of the robot on the application side if the robot has one
    QPair<QPair<QString, QString>, QStringList> appPathInfo = getPathFromFile(robotName);
    qDebug() << "MainWindow::updatePathInfo appPathinfo" << appPathInfo;
    QVector<PathPoint> robotPath = extractPathFromInfo(path);

    /// contains the groupname and pathname of the path described by the robot
    QPair<QString, QString> robotPathInApp = pathsController->findPath(robotPath);

    /// if the robot has a path
    if(robotPathInApp.first.compare("")){

        bool flag(false);
        pathsController->getPath(appPathInfo.first.first, appPathInfo.first.second, flag);

        /// the application also has a path for this robot
        if(flag){
            /// they have different paths so we keep the most recent one
            if(robotPathInApp.first.compare(appPathInfo.first.first) || robotPathInApp.second.compare(appPathInfo.first.second)){
                qDebug() << "mainWindow::updatepathinfo DIFFERENT PATHS";
                /// the file is more recent on the robot
                /// we do as if the application did not have a path at all
                if(isLater(pathDate.split("-", QString::SkipEmptyParts), appPathInfo.second)){
                    qDebug() << " BUT ROBOT MORE RECENT";
                    /// the application does not have a path so we use the path sent by the robot and update the file on the app side
                    bool foundFlag(true);
                    pathsController->getPathPainter()->setCurrentPath(pathsController->getPath(robotPathInApp.first, robotPathInApp.second, foundFlag), robotPathInApp.second);
                    robotsController->getEditSelectedRobotWidget()->setAssignedPath(robotPathInApp.second);
                    robotsController->getEditSelectedRobotWidget()->setGroupPath(robotPathInApp.first);
                    pathsController->setVisiblePath(robotPathInApp.second);
                    robotView->getRobot()->setPath(pathsController->getPathPainter()->getCurrentPath());
                    robotView->getRobot()->setGroupPathName(robotPathInApp.first);
                    robotView->getRobot()->setPathName(robotPathInApp.second);
                    int id = robotsController->getRobots()->getRobotId(robotView->getRobot()->getName());
                    bottomLayout->updateRobot(id, robotView);
                    if(pathsController->getPathPainter()->getCurrentPath().size() > 0){
                        bottomLayout->getViewPathRobotBtnGroup()->button(id)->setChecked(true);
                        viewPathSelectedRobot(id, true);
                    }
                    QFile fileInfo(QDir::currentPath() + QDir::separator() + "robotsController->getRobots()_paths" + QDir::separator()
                                   + robotName + "_path");
                    if(fileInfo.open(QIODevice::ReadWrite)){
                        fileInfo.resize(0);
                        QTextStream out(&fileInfo);
                        /// contains the date of the last modification of the path file on the robot
                        out << pathDate;
                        out << "%" << robotPathInApp.first << "%" << robotPathInApp.second;
                        qDebug() << "date now is" << pathDate;
                        fileInfo.close();
                    }

                } else {
                    qDebug() << "BUT APP MORE RECENT";
                    /// the file is more recent on the application side
                    /// we sent the path to the robot
                    /// but the application has one
                    bool tmpFlag(false);
                    Paths::Path currPath = pathsController->getPath(appPathInfo.first.first, appPathInfo.first.second, tmpFlag);
                    QString pathStr = prepareCommandPath(currPath);
                    commandController->sendCommand(robotView->getRobot(), QString("i ") + pathStr, "", appPathInfo.first.first, appPathInfo.first.second, false, -1, path);
                }
            } else {
                /// they have the same path
                qDebug() << "mainWindow::updatepathinfo SAME PATH";
                bool foundFlag(true);
                pathsController->getPathPainter()->setCurrentPath(pathsController->getPath(robotPathInApp.first, robotPathInApp.second, foundFlag), robotPathInApp.second);
                robotsController->getEditSelectedRobotWidget()->setAssignedPath(robotPathInApp.second);
                pathsController->setVisiblePath(robotPathInApp.second);
                robotsController->getEditSelectedRobotWidget()->setGroupPath(robotPathInApp.first);
                robotView->getRobot()->setPath(pathsController->getPathPainter()->getCurrentPath());
                robotView->getRobot()->setGroupPathName(robotPathInApp.first);
                robotView->getRobot()->setPathName(robotPathInApp.second);
                int id = robotsController->getRobots()->getRobotId(robotView->getRobot()->getName());
                bottomLayout->updateRobot(id, robotView);
                if(pathsController->getPathPainter()->getCurrentPath().size() > 0){
                    bottomLayout->getViewPathRobotBtnGroup()->button(id)->setChecked(true);
                    viewPathSelectedRobot(id, true);
                }
            }
        } else {
            qDebug() << "mainWindow::updatepathinfo ONLY ROBOT HAS A PATH";
            /// the application does not have a path so we use the path sent by the robot and update the file on the app side
            bool foundFlag(true);
            pathsController->getPathPainter()->setCurrentPath(pathsController->getPath(robotPathInApp.first, robotPathInApp.second, foundFlag), robotPathInApp.second);
            robotsController->getEditSelectedRobotWidget()->setAssignedPath(robotPathInApp.second);
            pathsController->setVisiblePath(robotPathInApp.second);
            robotsController->getEditSelectedRobotWidget()->setGroupPath(robotPathInApp.first);
            robotView->getRobot()->setPath(pathsController->getPathPainter()->getCurrentPath());
            robotView->getRobot()->setGroupPathName(robotPathInApp.first);
            robotView->getRobot()->setPathName(robotPathInApp.second);
            int id = robotsController->getRobots()->getRobotId(robotView->getRobot()->getName());
            bottomLayout->updateRobot(id, robotView);
            if(pathsController->getPathPainter()->getCurrentPath().size() > 0){
                bottomLayout->getViewPathRobotBtnGroup()->button(id)->setChecked(true);
                viewPathSelectedRobot(id, true);
            }
            QFile fileInfo(QDir::currentPath() + QDir::separator() + "robotsController->getRobots()_paths" + QDir::separator()
                           + robotName + "_path");
            if(fileInfo.open(QIODevice::ReadWrite)){
                fileInfo.resize(0);
                QTextStream out(&fileInfo);
                QString currentDateTime = QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss");
                /// contains the date of the last modification of the path file on the robot
                out << pathDate;
                out << "%" << robotPathInApp.first << "%" << robotPathInApp.second;
                qDebug() << "date now is" << pathDate;
                fileInfo.close();
            }
        }
    } else {
        /// the robot does not have a path
        bool flag(false);
        Paths::Path currPath = pathsController->getPath(appPathInfo.first.first, appPathInfo.first.second, flag);
        if(flag){
            qDebug() << "mainWindow::updatepathinfo ONLY APP HAS A PATH";
            /// but the application has one
            /// prepares the cmd to send to the robot

            QString pathStr = prepareCommandPath(currPath);
            commandController->sendCommand(robotView->getRobot(), QString("i ") + pathStr, "", appPathInfo.first.first, appPathInfo.first.second, false, -1, path);
        } else
           qDebug() << "mainWindow::updatepathinfo NO PATH ON EITHER SIDE";
    }
}

QPair<QPair<QString, QString>, QStringList> MainWindow::getPathFromFile(const QString robotName){
    /// QPair<QPair<groupName, pathName>, date>
    QPair<QPair<QString, QString>, QStringList> pathInfo;
    QFile fileInfo(QDir::currentPath() + QDir::separator() + "robotsController->getRobots()_paths" + QDir::separator() + robotName + "_path");
    if(fileInfo.open(QIODevice::ReadWrite)){
        QRegExp regex("[-\n%]");
        QString content = fileInfo.readAll();
        QStringList l = content.split(regex, QString::SkipEmptyParts);
        qDebug() << "path QStringlist" << l;
        if(l.size() == 8){
            for(int i = 0; i < 6; i++)
                pathInfo.second.push_back(l.at(i));
            pathInfo.first.first = l.at(6);
            pathInfo.first.second = l.at(7);
        }
    }
    fileInfo.close();
    return pathInfo;
}

QString MainWindow::prepareCommandPath(const Paths::Path &path) const {
    QString pathStr("");
    for(int i = 0; i < path.size(); i++){
        QSharedPointer<PathPoint> pathPoint = path.at(i);
        float oldPosX = pathPoint->getPoint().getPosition().getX();
        float oldPosY = pathPoint->getPoint().getPosition().getY();

        Position posInRobotCoordinates = Helper::Convert::pixelCoordToRobotCoord(Position(oldPosX, oldPosY), mapController->getMapOrigin().getX(), mapController->getMapOrigin().getY(), mapController->getMapResolution(), mapController->getMapHeight());

        int waitTime = pathPoint->getWaitTime();

        pathStr += + "\"" + QString::number(posInRobotCoordinates.getX()) + "\" \"" + QString::number(posInRobotCoordinates.getY()) + "\" \"" + QString::number(waitTime)+ "\" ";
    }

    return pathStr;
}

void MainWindow::testFunctionSlot(){
    qDebug() << "MainWindow::testFunctionSlot called";
    scanMapSlot();
    //openPositionRecoveryWidget();
}

void MainWindow::switchFocus(const QString name, QWidget* widget, const MainWindow::WidgetType type){
    lastWidgets.append(QPair<QPair<QWidget*, QString>, MainWindow::WidgetType>(QPair<QWidget*, QString>(widget,name), type));

    (lastWidgets.size() > 1) ? leftMenu->showBackButton(lastWidgets.at(lastWidgets.size()-2).first.second) :
                               leftMenu->hideBackButton();
}

void MainWindow::resetFocus(){
    lastWidgets = QList<QPair<QPair<QWidget*,QString>, MainWindow::WidgetType>>();
    updateView();
}

void MainWindow::updateView(){
    if(leftMenu != NULL)
        (lastWidgets.size() <= 1) ? leftMenu->hideBackButton() : leftMenu->showBackButton(lastWidgets.last().first.second);
}

void MainWindow::openLeftMenu(){
    qDebug() << "openLeftMenu called";
    topLayoutController->setLabel(TEXT_COLOR_NORMAL, "");

    /// resets the color of the selected point on the map and hides the temporary point`
    if(pointsController->getDisplaySelectedPoint()->getPointView()){
        QSharedPointer<PointView> displaySelectedPointView = pointsController->getPoints()->findPointView(pointsController->getDisplaySelectedPoint()->getPointName());
        if(displaySelectedPointView)
            displaySelectedPointView->setPixmap(PointView::PixmapType::NORMAL);
    }

    resetFocus();

    if(leftMenu->isHidden()){

        hideAllWidgets();
        leftMenuWidget->show();
        leftMenu->show();
        switchFocus("Menu", leftMenuWidget, MainWindow::WidgetType::MENU);

    } else {
        pointsController->getDisplaySelectedPoint()->hide();
        if(leftMenuWidget->isHidden()){
            hideAllWidgets();
            leftMenuWidget->show();
            leftMenu->show();
            switchFocus("Menu",leftMenuWidget, MainWindow::WidgetType::MENU);
        } else
            closeSlot();
    }
}

void MainWindow::activateLaserSlot(QString name, bool activate){
    QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByName(name);
    if(robotView && robotView->getRobot()){
        robotView->setObstacles(QVector<QPointF>());
        if(activate)
            commandController->sendCommand(robotView->getRobot(), QString("q"));
        else
            commandController->sendCommand(robotView->getRobot(), QString("r"));
    } else {
        qDebug() << "MainWindow::activateLaserSlot wants to activate the laser of an unknown robot on ip" << name;
        Q_UNREACHABLE();
    }
}

void MainWindow::closeEvent(QCloseEvent *event){
    qDebug() << "MainWindow::closeEvent";

    if(mapController->hasMapChanged()){
        QMessageBox msgBox;
        msgBox.setText("The map has been modified.");
        msgBox.setInformativeText("Do you want to save your changes ?");
        msgBox.setStandardButtons(QMessageBox::Save | QMessageBox::Cancel | QMessageBox::Discard);
        msgBox.setDefaultButton(QMessageBox::Save);
        int ret = msgBox.exec();
        switch (ret) {
            case QMessageBox::Save:
                saveMapBtnEvent();
                closeWidgets();
                QMainWindow::closeEvent(event);
            break;
            case QMessageBox::Discard:
                closeWidgets();
                QMainWindow::closeEvent(event);
            break;
            case QMessageBox::Cancel:
                event->ignore();
            break;
            default:
                closeWidgets();
                QMainWindow::closeEvent(event);
            break;
        }
    } else {
        closeWidgets();
        QMainWindow::closeEvent(event);
    }
}

void MainWindow::closeWidgets(){
    qDebug() << "MainWindow::closeWidgets";
    emit stopAllCmd();

    if(mergeMapWidget)
        mergeMapWidget->close();

    if(scanMapWidget)
        scanMapWidget->close();

}

void MainWindow::getMapForMergingSlot(QString robotName){
    QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByName(robotName);
    if(robotView && robotView->getRobot())
        commandController->sendCommand(robotView->getRobot(), QString("s \"2\""));
}

void MainWindow::openHelpMessage(const QString message, const QString feature){
    qDebug() << "MainWindow::openHelpMessage called with feature" << feature;
    currentFeature = feature;
    if(settingsController->getSettings()->getHelpNeeded(feature)) {
        QMessageBox box;
        QCheckBox* checkbox = new QCheckBox("Never show this again (can be reset in settings)");

        /// sends a signal to the settings controller to enable or disable a message for a particular feature
        connect(checkbox, SIGNAL(toggled(bool)), this, SLOT(relayTutorialSignal(bool)));

        /// SLOT that is actually going to modify the current settings
        connect(this, SIGNAL(tutorialSignal(bool, QString)), settingsController, SLOT(hideTutorial(bool, QString)));

        box.setText(message);
        box.addButton(QMessageBox::Ok);
        box.setCheckBox(checkbox);
        box.exec();
    }
}

void MainWindow::relayTutorialSignal(const bool messageNeeded){
    qDebug() << "emitting tuto signal for feature " << currentFeature;
    emit tutorialSignal(messageNeeded, currentFeature);
}

void MainWindow::commandDoneSlot(QString cmdName, bool success, QString robotName, QString newRobotName, QString groupName, QString pathName, bool scan, int nb, QStringList path){
    //qDebug() << "MainWindow::commandDoneSlot" << cmdName << success << newRobotName << groupName << pathName << scan << nb << path;

    if(!cmdName.isEmpty()){
        switch (cmdName.at(0).unicode()) {
            case 'a':
                /// Changed the name of the robot
                commandDoneNewName(success, newRobotName);
            break;
            case 'b':
                /// Changed the wifi informations of a robot
                /// OSEF
            break;
            case 'c':
                /// Sent the robot to a new goal
                /// OSEF
            break;
            case 'd':
                /// Paused the path of the robot
                commandDonePausePath(success, robotName);
            break;
            case 'e':
                /// Played the scan of the map
                commandDonePlayScan(success, scan, robotName);
            break;
            case 'f':
                /// Paused the scan of the map
                commandDonePauseScan(success, scan, robotName);
            break;
            case 'g':
                /// Updated the name & wifi of the robot
                /// OSEF
            break;
            case 'h':
                /// Sent the ports to the robot
                /// OSEF
            break;
            case 'i':
                /// Sent a new path to the robot
                commandDoneSendPath(success, scan, robotName, groupName, pathName, path);
            break;
            case 'j':
                /// Played the path of the robot
                commandDonePlayPath(success, robotName);
            break;
            case 'k':
                /// Deleted the path of the robot
                commandDoneDeletePath(success, robotName);
            break;
            case 'l':
                /// Stopped the path of the robot
                commandDoneStopPath(success, robotName);
            break;
            case 'm':
                /// Stopped and deleted the path of the robot
                commandDoneStopDeletePath(success, robotName);
            break;
            case 'n':
                /// Sent the new home to the robot
                commandDoneNewHome(success, robotName, nb, newRobotName);
            break;
            case 'o':
                /// Sent the robot to its home
                commandDoneGoHome(success, robotName);
            break;
            case 'p':
                /// NOT USED
                /// Stopped the robot to go home
                Q_UNREACHABLE();
            break;
            case 'q':
                /// Started the laser of the robot
                /// OSEF
            break;
            case 'r':
                /// Stopped the laser of the robot
                /// OSEF
            break;
            case 's':
                /// Received the map from the robot
                /// OSEF
            break;
            case 't':
                /// Started a new scan
                commandDoneStartScan(success, scan, robotName);
            break;
            case 'u':
                /// Stopped the current scan
                commandDoneStopScan(success, robotName);
            break;
            default:
                /// Unknown/unused command or we simply don't care
            break;
        }
    }
}

void MainWindow::commandDoneNewName(bool success, QString name){
    if(success){
        CustomRobotDialog* robotDialog = robotsController->getEditSelectedRobotWidget()->getRobotInfoDialog();

        /// updates the name of the file which stores the path of the robot
        QFile robotPathFile(QDir::currentPath() + QDir::separator() + "robotsController->getRobots()_paths" + QDir::separator() + robotsController->getSelectedRobot()->getRobot()->getName() + "_path");
        if(robotPathFile.exists())
            robotPathFile.rename(QDir::currentPath() + QDir::separator() + "robotsController->getRobots()_paths" + QDir::separator() + name + "_path");

        /// updates the name of the file which stores the home of the robot
        QFile robotHomeFile(QDir::currentPath() + QDir::separator() + "robotsController->getRobots()_homes" + QDir::separator() + robotsController->getSelectedRobot()->getRobot()->getName());
        if(robotHomeFile.exists())
            robotHomeFile.rename(QDir::currentPath() + QDir::separator() + "robotsController->getRobots()_homes" + QDir::separator() + name);

        QMap<QString, QString> tmp = robotsController->getRobots()->getRobotsNameMap();
        tmp[robotsController->getSelectedRobot()->getRobot()->getIp()] = name;
        robotsController->getSelectedRobot()->getRobot()->setName(name);
        robotsController->getRobots()->setRobotsNameMap(tmp);

        emit changeCmdThreadRobotName(name);
        QFile fileWrite(QDir::currentPath() + QDir::separator() + QString(ROBOTS_NAME_FILE));
        fileWrite.resize(0);
        fileWrite.open(QIODevice::WriteOnly);
        QDataStream out(&fileWrite);
        out << robotsController->getRobots()->getRobotsNameMap();
        fileWrite.close();

        qDebug() << "MainWindow::robotSavedEvent RobotsNameMap updated" << robotsController->getRobots()->getRobotsNameMap();
        bottomLayout->updateRobot(robotsController->getRobots()->getRobotId(robotsController->getSelectedRobot()->getRobot()->getName()), robotsController->getSelectedRobot());
        robotsController->getEditSelectedRobotWidget()->getNameLabel()->setText(name);
        robotsController->getEditSelectedRobotWidget()->getRobotInfoDialog()->getNameEdit()->setText(name);

        robotsController->getRobotsLeftWidget()->updateRobots(robotsController->getRobots());
        robotDialog->getNameEdit()->setText(robotsController->getSelectedRobot()->getRobot()->getName());
        topLayoutController->setLabel(TEXT_COLOR_SUCCESS, "You have successfully updated" + name);
    } else
        topLayoutController->setLabel(TEXT_COLOR_DANGER, "Failed to edit the name of the robot, please try again");
}

void MainWindow::commandDonePausePath(bool success, QString robotName){
    QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByName(robotName);
    int robotNb = robotsController->getRobots()->getRobotId(robotName);
    if(robotView && robotNb >= 0){
        if(success){
            robotView->getRobot()->setPlayingPath(false);
            bottomLayout->getPlayRobotBtnGroup()->button(robotNb)->setIcon(QIcon(":/icons/play.png"));

            if(abs(robotView->getLastStage()) > 0)
                bottomLayout->getStopRobotBtnGroup()->button(robotNb)->setEnabled(true);
            else
                bottomLayout->getStopRobotBtnGroup()->button(robotNb)->setEnabled(false);
            topLayoutController->setLabel(TEXT_COLOR_SUCCESS, "Path paused");
        } else
            topLayoutController->setLabel(TEXT_COLOR_DANGER, "Path failed to be paused, please try again");
    }
}

void MainWindow::commandDonePlayScan(bool success, bool scan, QString robotName){
    if(success)
        emit robotScanning(scan, robotName, true);
    else
        emit robotScanning(scan, robotName, false);
}

void MainWindow::commandDonePauseScan(bool success, bool scan, QString robotName){
    if(success)
        emit robotScanning(scan, robotName, true);
    else
        emit robotScanning(scan, robotName, false);
}

void MainWindow::commandDoneSendPath(bool success, bool boolean, QString robotName, QString groupName, QString pathName, QStringList path){
    QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByName(robotName);

    if(robotView){
        bool flag(false);
        if(boolean){
            if(success){
                QVector<PathPoint> robotPath = extractPathFromInfo(path);
                QPair<QString, QString> robotPathInApp = pathsController->findPath(robotPath);
                pathsController->getPathPainter()->setCurrentPath(pathsController->getPath(groupName, pathName, flag), pathName);
                robotsController->getEditSelectedRobotWidget()->setAssignedPath(pathName);
                pathsController->setVisiblePath(robotPathInApp.second);
                robotsController->getEditSelectedRobotWidget()->setGroupPath(groupName);
                robotView->getRobot()->setPath(pathsController->getPathPainter()->getCurrentPath());
                robotView->getRobot()->setGroupPathName(groupName);
                robotView->getRobot()->setPathName(pathName);
                int id = robotsController->getRobots()->getRobotId(robotName);
                bottomLayout->updateRobot(id, robotView);
                if(pathsController->getPathPainter()->getCurrentPath().size() > 0){
                    bottomLayout->getViewPathRobotBtnGroup()->button(id)->setChecked(true);
                    viewPathSelectedRobot(id, true);
                }
            }
        } else {
            if(success){
                /// we update the path on the application side by serializing the path
                QFile fileInfo(QDir::currentPath() + QDir::separator() + "robotsController->getRobots()_paths" + QDir::separator() + robotView->getRobot()->getName() + "_path");
                if(fileInfo.open(QIODevice::ReadWrite)){
                    fileInfo.resize(0);
                    QTextStream out(&fileInfo);
                    QString currentDateTime = QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss");
                    out << currentDateTime;
                    out << "%" << groupName << "%" << pathName;
                    qDebug() << "date now is" << currentDateTime;
                    fileInfo.close();
                    robotsController->getEditSelectedRobotWidget()->setPath(pathsController->getPath(groupName, pathName, flag));
                    robotsController->getEditSelectedRobotWidget()->setGroupPath(groupName);
                    robotsController->getEditSelectedRobotWidget()->setAssignedPath(pathName);
                    emit updatePath(groupName, pathName);
                }
                topLayoutController->setLabel(TEXT_COLOR_SUCCESS, "the path of \"" + robotView->getRobot()->getName() + "\" has been successfully updated");
                qDebug() << "MainWindow::sendPathSelectedRobotSlot Path saved for robot" << robotView->getRobot()->getIp();
            } else {
                topLayoutController->setLabel(TEXT_COLOR_DANGER, "The path of " + robotView->getRobot()->getName() + "\" could not be updated, please try again");
                qDebug() << "MainWindow::sendPathSelectedRobotSlot Path failed to be saved, please try again";
            }
        }
    }
}

void MainWindow::commandDonePlayPath(bool success, QString robotName){
    QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByName(robotName);
    int robotNb = robotsController->getRobots()->getRobotId(robotName);
    if(robotView && robotNb >= 0){
        if(success){
            robotView->getRobot()->setPlayingPath(true);
            bottomLayout->getPlayRobotBtnGroup()->button(robotNb)->setIcon(QIcon(":/icons/pause.png"));
            bottomLayout->getStopRobotBtnGroup()->button(robotNb)->setEnabled(true);
            topLayoutController->setLabel(TEXT_COLOR_SUCCESS, "Path playing");
        } else
            topLayoutController->setLabel(TEXT_COLOR_DANGER, "Path failed to start, please try again");
    }
}

void MainWindow::commandDoneDeletePath(bool success, QString robotName){
    QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByName(robotName);
    int robotNb = robotsController->getRobots()->getRobotId(robotName);
    if(robotView && robotNb >= 0){
        if(success){
            clearPath(robotNb);
            topLayoutController->setLabel(TEXT_COLOR_SUCCESS, "The path of \"" + robotView->getRobot()->getName() + "\" has been successfully deleted");
        } else
            topLayoutController->setLabel(TEXT_COLOR_DANGER, "Failed to delete the path of " + robotView->getRobot()->getName() + ", please try again");
    }
}

void MainWindow::commandDoneStopPath(bool success, QString robotName){
    QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByName(robotName);
    int robotNb = robotsController->getRobots()->getRobotId(robotName);
    if(robotView && robotNb >= 0){
        if(success){
            robotView->getRobot()->setPlayingPath(false);
            bottomLayout->getPlayRobotBtnGroup()->button(robotNb)->setIcon(QIcon(":/icons/play.png"));
            bottomLayout->getStopRobotBtnGroup()->button(robotNb)->setEnabled(false);
            topLayoutController->setLabel(TEXT_COLOR_SUCCESS, "Path stopped");
        } else
            topLayoutController->setLabel(TEXT_COLOR_DANGER, "Path failed to be stopped, please try again");
    }
}

void MainWindow::commandDoneStopDeletePath(bool success, QString robotName){
    QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByName(robotName);
    int robotNb = robotsController->getRobots()->getRobotId(robotName);
    if(robotView && robotNb >= 0){
        if(success){
            clearPath(robotNb);
            topLayoutController->setLabel(TEXT_COLOR_SUCCESS, "The path of " + robotView->getRobot()->getName() + " has been successfully deleted");
        } else
            topLayoutController->setLabel(TEXT_COLOR_DANGER, "Failed to delete the path of " + robotView->getRobot()->getName() + ", please try again");
    }
}

void MainWindow::commandDoneNewHome(bool success, QString robotName, int id, QString homeName){
    switch(id){
        case 0:
            if(success){

                QSharedPointer<PointView> home = pointsController->getPoints()->findPointView(homeName);
                robotsController->getEditSelectedRobotWidget()->getGoHomeBtn()->show();

                /// Remove the previous home
                if(robotsController->getEditSelectedRobotWidget()->getHome()){
                    robotsController->getEditSelectedRobotWidget()->getHome()->getPoint()->setHome(Point::PERM);
                    robotsController->getEditSelectedRobotWidget()->getHome()->setPixmap(PointView::PixmapType::NORMAL);
                    robotsController->getEditSelectedRobotWidget()->getHome()->getPoint()->setRobotName("");
                }

                pointsController->savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

                /// associates the robot to the point
                home->getPoint()->setRobotName(robotsController->getSelectedRobot()->getRobot()->getName());
                home->getPoint()->setHome(Point::HOME);

                /// saves the position of the new home in the corresponding file
                QFileInfo homeFileInfo(QDir::currentPath(), "../gobot-software/robotsController->getRobots()_homes/" + robotsController->getSelectedRobot()->getRobot()->getName());
                std::ofstream homeFile(homeFileInfo.absoluteFilePath().toStdString(), std::ios::out | std::ios::trunc);
                if(homeFile){
                    homeFile << home->getPoint()->getPosition().getX() << " " << home->getPoint()->getPosition().getY();
                    homeFile.close();
                }
                else
                    qDebug() << "could not save home";

                robotsController->getEditSelectedRobotWidget()->setHome(home);
                robotsController->getSelectedRobot()->getRobot()->setHome(home);
                robotsController->getEditSelectedRobotWidget()->updateHomeMenu();

                topLayoutController->removeRobotWithoutHome(robotsController->getSelectedRobot()->getRobot()->getName());

                /// so that if the new home if part of the path it displays the path correctly (not a house on top of a normal point)
                /// this call makes the home

                pathsController->getPathPainter()->setCurrentPath(robotsController->getSelectedRobot()->getRobot()->getPath(), "");

                /// setCurrentPath is displaying the path so if it was not displayed we hide it
                if(!bottomLayout->getViewPathRobotBtnGroup()->button(robotsController->getRobots()->getRobotId(robotsController->getSelectedRobot()->getRobot()->getName()))->isChecked())
                    emit resetPath();

                pointsController->showHomeFromHomeName(robotsController->getSelectedRobot()->getRobot()->getHome()->getPoint()->getName());

                home->setPixmap(PointView::PixmapType::SELECTED);
                home->show();

                topLayoutController->setLabel(TEXT_COLOR_SUCCESS, robotsController->getSelectedRobot()->getRobot()->getName() + " successfully updated its home point");
            } else
                topLayoutController->setLabel(TEXT_COLOR_DANGER, robotsController->getSelectedRobot()->getRobot()->getName() + " failed to save its home point, please try again");

        break;
        case 1:
            if(success){
                QSharedPointer<PointView> home = pointsController->getPoints()->findPointView(homeName);
                updateHomeFile(home->getPoint()->getRobotName(),
                               home->getPoint()->getPosition(),
                               QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss").split("-"));
            }
        break;
        case 2:
            if(success)
                setHomeAtConnection(robotName, getHomeFromFile(robotName).first);
        break;
        default:
            Q_UNREACHABLE();
        break;
    }
}

void MainWindow::commandDoneGoHome(bool success, QString robotName){
    if(success)
        topLayoutController->setLabel(TEXT_COLOR_SUCCESS, "The robot " + robotName + " is going home");
    else
        topLayoutController->setLabel(TEXT_COLOR_DANGER, "Failed to send the robot " + robotName + " home, please try again");
}

void MainWindow::commandDoneStartScan(bool success, bool scan, QString robotName){
    QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByName(robotName);
    if(robotView){
        if(scan){
            if(success){
                emit startedScanning(robotName, true);
                robotView->getRobot()->setScanning(true);
            } else
                emit startedScanning(robotName, false);
        } else {
            if(success){
                emit robotScanning(scan, robotName, true);
                robotView->getRobot()->setScanning(true);
            } else
                emit robotScanning(scan, robotName, false);
        }
    }
}

void MainWindow::commandDoneStopScan(bool success, QString robotName){
    QPointer<RobotView> robotView = robotsController->getRobots()->getRobotViewByName(robotName);
    if(robotView){
        if(success){
            robotView->getRobot()->setScanning(false);
            qDebug() << "MainWindow::stopScanningSlot Successfully stopped the robot" << robotName << "to scan";
        } else
            qDebug() << "MainWindow::stopScanningSlot Could not stop the robot" << robotName << "to scan, stopped trying after 5 attempts";
    }
}

void MainWindow::testCoordSlot(double x, double y){
    qDebug() << "MainWindow::testCoordSlot Trying to go to" << x << y;
    Position posInRobotCoordinates = Helper::Convert::pixelCoordToRobotCoord(Position(x, y), mapController->getMap()->getOrigin().getX(), mapController->getMap()->getOrigin().getY(), mapController->getMap()->getResolution(), mapController->getMap()->getHeight());
    qDebug() << "MainWindow::testCoordSlot converted in robot coord to" << posInRobotCoordinates.getX() << posInRobotCoordinates.getY();
}
