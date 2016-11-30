#include "mainwindow.h"
#include <QMap>
#include <QVBoxLayout>
#include <QAbstractButton>
#include <QString>
#include <QStringList>
#include <QVector>
#include <assert.h>
#include "ui_mainwindow.h"
#include "Controller/robotserverworker.h"
#include "Model/pathpoint.h"
#include "Model/map.h"
#include "Model/robots.h"
#include "Model/robot.h"
#include "Model/xmlparser.h"
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
#include "View/toplayout.h"
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
#include <QtGlobal>
#include <QStringList>
#include "Controller/commandcontroller.h"
#include <fstream>
#include "View/editmapwidget.h"

#include <chrono>
#include <thread>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    /// centers the msgBox on the middle of the screen
    msgBox.move(mapToGlobal(QPoint(QApplication::desktop()->screenGeometry().width()/2,
                                   QApplication::desktop()->screenGeometry().height()/2) ));

    /// initializes the paths and points used in the application
    points = QSharedPointer<Points>(new Points(this));
    paths = QSharedPointer<Paths>(new Paths());

    QWidget* mainWidget = new QWidget(this);

    QVBoxLayout* mainLayout = new QVBoxLayout(mainWidget);

    /// initializes the map used by the application
    map = QSharedPointer<Map>(new Map());


    //qDebug() << "Current time :" << QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss");
    //qDebug() << "Map Id :" << QUuid::createUuid().toString();

    std::ifstream file((QDir::currentPath() + QDir::separator() + "currentMap.txt").toStdString(), std::ios::in);

    if(file){
        int _height, _width;
        double centerX, centerY, originX, originY, resolution;
        std::string dateTime, mapId;
        file >> mapFile >> _height >> _width >> centerX >> centerY >> mapState.second >> originX >> originY >> resolution >> dateTime >> mapId;
        qDebug() << "CurrentMap.txt :" << QString::fromStdString(mapFile) << _height << _width
                 << centerX << centerY << originX << originY << resolution
                 << QString::fromStdString(dateTime) << QString::fromStdString(mapId);
        map->setHeight(_height);
        map->setWidth(_width);
        mapState.first.setX(centerX);
        mapState.first.setY(centerY);
        map->setOrigin(Position(originX, originY));
        map->setResolution(resolution);
        map->setDateTime(QDateTime::fromString(QString::fromStdString(dateTime), "yyyy-MM-dd-hh-mm-ss"));
        map->setMapId(QUuid(QString::fromStdString(mapId)));
        file.close();
    }

    if(!mapFile.compare("")){
        setWindowTitle(":/maps/map.pgm");
        map->setMapFromFile(":/maps/map.pgm");
        mapState.second = 1;
        map->setHeight(608);
        map->setWidth(320);
        mapState.first.setX(0);
        mapState.first.setY(0);
        map->setResolution(0.05);
        map->setOrigin(Position(0, 0));
        map->setDateTime(QDateTime(QDate::fromString("1970-01-01", "yyyy-mm-dd")));
        map->setMapId(QUuid());
    } else {
        setWindowTitle(QString::fromStdString(mapFile));
        map->setMapFromFile(QString::fromStdString(mapFile));
    }

    /// retrieves the configuration of the map from the settings file
    /// if the parameters are not there, .0f, .0f and 1.0f serve as the
    /// default parameter

    scene = new QGraphicsScene();

    graphicsView = new CustomQGraphicsView(scene, this);

    selectedRobot = NULL;
    scanningRobot = NULL;
    selectedPoint = QSharedPointer<PointView>();
    editedPointView = QSharedPointer<PointView>();
    robotServerWorker = NULL;
    commandController = new CommandController(this);

    robots = QSharedPointer<Robots>(new Robots());

    /// Create the graphic item of the map
    QPixmap pixmap = QPixmap::fromImage(map->getMapImage());
    mapPixmapItem = new MapView(pixmap, QSize(geometry().width(), geometry().height()), map, this, robots);

    /// Create the toolbar
    topLayout = new TopLayout(this);
    mainLayout->addWidget(topLayout);

    QHBoxLayout* bottom = new QHBoxLayout();

    initializePaths();

    initializePoints();

    /// to draw stand-alone paths
    pathPainter = new PathPainter(this, points);

    initializeRobots();

    scene->setSceneRect(0, 0, 800, 600);

    scene->addItem(mapPixmapItem);

    graphicsView->scale(std::max(graphicsView->parentWidget()->width()/scene->width(), graphicsView->parentWidget()->height()/scene->height()),
                        std::max(graphicsView->parentWidget()->width()/scene->width(), graphicsView->parentWidget()->height()/scene->height()));

    /// hides the scroll bars
    graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    graphicsView->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));

    leftMenu = new LeftMenu(this, points, paths, robots, map, pathPainter);

    resetFocus();
    initializeLeftMenu();
    bottom->addWidget(leftMenu);
    leftMenu->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

    rightLayout = new QVBoxLayout();
    bottom->addLayout(rightLayout);
    rightLayout->addWidget(graphicsView);

    initializeBottomPanel();

    graphicsView->show();

    ///      ------------------                   POINTS MENUS CONNECTS     ------------------------------------------------

    /// to link the map and the point information menu when a point is being edited
    connect(mapPixmapItem, SIGNAL(newCoordinates(double, double)), this, SLOT(updateCoordinates(double, double)));

    /// to cancel the modifications on an edited point
    connect(leftMenu->getDisplaySelectedPoint()->getCancelButton(), SIGNAL(clicked(bool)), this, SLOT(cancelEvent()));

    /// to save the modifications on an edited point
    connect(leftMenu->getDisplaySelectedPoint()->getSaveButton(), SIGNAL(clicked(bool)), this, SLOT(updatePoint()));

    /// the purpose of this connection is just to propagate the signal to the map view through the main window
    connect(leftMenu->getDisplaySelectedPoint(), SIGNAL(nameChanged(QString, QString)), this, SLOT(updatePoint()));

    /// to reset the state of everybody when a user click on a random button while he was editing a point
    connect(leftMenu->getDisplaySelectedPoint(), SIGNAL(resetState(GraphicItemState)),  this, SLOT(setGraphicItemsState(GraphicItemState)));

    /// to connect the buttons in the left menu so they can be double clicked after they were updated
    connect(pointsLeftWidget->getGroupButtonGroup(), SIGNAL(updateConnectionsRequest()), this, SLOT(reestablishConnectionsGroups()));

    /// to connect the buttons in the group menu so they can be double clicked after they were updated
    connect(leftMenu->getDisplaySelectedGroup()->getPointButtonGroup(), SIGNAL(updateConnectionsRequest()), this, SLOT(reestablishConnectionsPoints()));

    /// to create a new group, the signal is sent by pointsLeftWidget
    connect(pointsLeftWidget, SIGNAL(newGroup(QString)), this, SLOT(createGroup(QString)));

    /// to modify the name of a group, the signal is sent by pointsLeftWidget
    connect(pointsLeftWidget, SIGNAL(modifiedGroup(QString)), this, SLOT(modifyGroupWithEnter(QString)));

    /// same but this happens when the user clicks on a random point of the window
    connect(pointsLeftWidget, SIGNAL(modifiedGroupAfterClick(QString)), this, SLOT(modifyGroupAfterClick(QString)));

    /// to know what message to display when a user is creating a group
    connect(pointsLeftWidget, SIGNAL(messageCreationGroup(QString, QString)), this, SLOT(setMessageCreationGroup(QString, QString)));


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ///  ------------------------------------------------------- PATHS CONNECTS ----------------------------------------------------------

    /// to link the map and the path information when a path point is being edited (associated to a robot or not)
    connect(mapPixmapItem, SIGNAL(newCoordinatesPathPoint(double, double)), this, SLOT(updateEditedPathPoint(double, double)));

    /// to link the map and the coordinates where we want to send the robot
    connect(mapPixmapItem, SIGNAL(newScanningGoal(double, double)), this, SLOT(newScanningGoalSlot(double, double)));

    /// to know what message to display when a user is creating a path
    connect(mapPixmapItem, SIGNAL(newMessage(QString)), this, SLOT(setMessageCreationPath(QString)));

    /// to add a path point when we click on the map
    connect(mapPixmapItem, SIGNAL(addPathPoint(QString, double, double)), pathCreationWidget, SLOT(addPathPointSlot(QString, double, double)));

    /// to add a path point when we click on a pointView (which is relayed by the mainWindow)
    connect(this, SIGNAL(addPathPoint(QString, double, double)), pathCreationWidget, SLOT(addPathPointSlot(QString, double, double)));

    connect(this, SIGNAL(updatePathPainter(bool)), pathPainter, SLOT(updatePathPainterSlot(bool)));

    connect(pathCreationWidget, SIGNAL(editTmpPathPoint(int, QString, double, double)), this, SLOT(editTmpPathPointSlot(int, QString, double, double)));

    connect(this, SIGNAL(updatePathPainterPointView()), pathPainter, SLOT(updatePathPainterPointViewSlot()));

    connect(pathCreationWidget, SIGNAL(saveEditPathPoint()), this, SLOT(saveEditPathPointSlot()));

    connect(pathCreationWidget, SIGNAL(cancelEditPathPoint()), this, SLOT(cancelEditPathPointSlot()));

    connect(pathCreationWidget, SIGNAL(savePath()), this, SLOT(savePathSlot()));

    connect(this, SIGNAL(resetPath()), pathPainter, SLOT(resetPathSlot()));

    connect(this, SIGNAL(resetPathCreationWidget()), pathCreationWidget, SLOT(resetWidget()));

    connect(leftMenu->getGroupsPathsWidget(), SIGNAL(newPathGroup(QString)), this, SLOT(saveGroupPaths(QString)));
    connect(leftMenu->getGroupsPathsWidget(), SIGNAL(messageCreationGroup(QString,QString)), this, SLOT(setMessageCreationGroup(QString,QString)));
    connect(leftMenu->getGroupsPathsWidget(), SIGNAL(modifiedGroup(QString)), this, SLOT(modifyGroupPathsWithEnter(QString)));

    connect(pathCreationWidget->getCancelButton(), SIGNAL(clicked()), this, SLOT(cancelNoRobotPathSlot()));

    connect(leftMenu->getpathCreationWidget(), SIGNAL(codeEditPath(int)), this, SLOT(setMessageNoRobotPath(int)));

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    mainLayout->addLayout(bottom);
    //graphicsView->setStyleSheet("CustomQGraphicsView {background-color: " + background_map_view + "}");
    setCentralWidget(mainWidget);

    /// to navigate with the tab key
    setTabOrder(leftMenu->getReturnButton(), pointsLeftWidget->getActionButtons()->getPlusButton());

    /// Centers the map and initialize the map state

    centerMap();

    /// Some style

/*
    setStyleSheet("QWidget {border: 1px solid red}"
                  "QPushButton {border: 1px solid green}"
                  "QLabel {border: 1px solid blue}"
                  "QScrollArea {border: 1px solid yellow}");
*/

    setAutoFillBackground(true);

    rightLayout->setContentsMargins(0, 0, 0, 0);
    bottom->setContentsMargins(0, 0, 0, 0);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    topLayout->setContentsMargins(0, 0, 0, 0);
    bottomLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(0);
}

MainWindow::~MainWindow(){
    delete ui;

    if(editMapWidget)
        delete editMapWidget;

    if(robotServerWorker){
        emit stopUpdateRobotsThread();
        serverThread.quit();
        serverThread.wait();
    }
}

/**********************************************************************************************************************************/

//                                          ROBOTS and PATHS

/**********************************************************************************************************************************/


void MainWindow::initializeRobots(){

    /// Get the list of taken robot's name from the file
    QFile fileRead(QDir::currentPath() + QDir::separator() + "robotsName.dat");

    fileRead.open(QIODevice::ReadWrite);
    /// read the data serialized from the file
    QDataStream in(&fileRead);
    QMap<QString, QString> tmp;
    in >> tmp;
    robots->setRobotsNameMap(tmp);
    fileRead.close();

    robotServerWorker = new RobotServerWorker(PORT_ROBOT_UPDATE);

    connect(robotServerWorker, SIGNAL(robotIsAlive(QString, QString, QString, QString, QString, int)), this, SLOT(robotIsAliveSlot(QString, QString, QString, QString, QString, int)));
    connect(this, SIGNAL(stopUpdateRobotsThread()), robotServerWorker, SLOT(stopWorker()));

    connect(&serverThread, SIGNAL(finished()), robotServerWorker, SLOT(deleteLater()));
    serverThread.start();
    robotServerWorker->moveToThread(&serverThread);

   /*


    QFileInfo fileInfo(QDir::currentPath(), "../gobot-software/" + QString(ROBOTS_NAME_FILE));
    QFile fileWrite(fileInfo.absoluteFilePath());

    fileWrite.resize(0);
    fileWrite.open(QIODevice::WriteOnly);
    QDataStream out(&fileWrite);
    QMap<QString, QString> tmpMap = robots->getRobotsNameMap();

    QString robotIp1 = "localhost";
    QString robotName1 = tmpMap.value(robotIp1, "Roboty");

    QPointer<Robot> robot1(QPointer<Robot>(new Robot(this, paths, robotName1, robotIp1)));
    robot1->setWifi("Swaghetti Yolognaise");
    QPointer<RobotView> robotView1 = QPointer<RobotView>(new RobotView(robot1, mapPixmapItem));
    robotView1->setLastStage(2);

    connect(robotView1, SIGNAL(setSelectedSignal(QPointer<RobotView>)), this, SLOT(setSelectedRobot(QPointer<RobotView>)));

    robotView1->setPosition(896, 1094);
    robotView1->setParentItem(mapPixmapItem);
    robots->add(robotView1);
    tmpMap[robot1->getIp()] = robot1->getName();

    QString robotIp2 = "192.168.4.12";
    QString robotName2 = tmpMap.value(robotIp2, "Roboto");
    QPointer<Robot> robot2(QPointer<Robot>(new Robot(this, paths, robotName2, robotIp2)));
    robot2->setWifi("Swaghetti Yolognaise");
    QPointer<RobotView> robotView2 = QPointer<RobotView>(new RobotView(robot2, mapPixmapItem));
    connect(robotView2, SIGNAL(setSelectedSignal(QPointer<RobotView>)), this, SLOT(setSelectedRobot(QPointer<RobotView>)));
    robotView2->setPosition(907, 1175);
    robotView2->setParentItem(mapPixmapItem);
    robots->add(robotView2);
    tmpMap[robot2->getIp()] = robot2->getName();

    QString robotIp3 = "192.168.4.13";
    QString robotName3 = tmpMap.value(robotIp3, "Robota");
    QPointer<Robot> robot3(QPointer<Robot>(new Robot(this, paths, robotName3, robotIp3)));
    robot3->setWifi("Swaghetti Yolognaise");
    QPointer<RobotView> robotView3 = QPointer<RobotView>(new RobotView(robot3, mapPixmapItem));
    connect(robotView3, SIGNAL(setSelectedSignal(QPointer<RobotView>)), this, SLOT(setSelectedRobot(QPointer<RobotView>)));
    robotView3->setPosition(1148, 915);
    robotView3->setParentItem(mapPixmapItem);
    robots->add(robotView3);
    tmpMap[robot3->getIp()] = robot3->getName();

    robots->setRobotsNameMap(tmpMap);
    out << robots->getRobotsNameMap();
    fileWrite.close();


    for(int i = 0; i < robots->getRobotsVector().size(); i++){
        QFileInfo fileinfo(QDir::currentPath(), "../gobot-software/robots_paths/" + robots->getRobotsVector().at(i)->getRobot()->getName() + "_path");
        QFile robotPathFile(fileinfo.absoluteFilePath());
        qDebug() << "robot file" << fileinfo.absoluteFilePath();
        if(robotPathFile.exists()){
            robotPathFile.open(QIODevice::ReadOnly);
            QDataStream in(&robotPathFile);
            in >> *(robots->getRobotsVector().at(i)->getRobot());
            robotPathFile.close();
        }
    }
*/
    //qDebug() << "RobotsNameMap on init" << robots->getRobotsNameMap();
}

void MainWindow::updateRobot(const QString ipAddress, const float posX, const float posY, const float oriZ){

    float newPosX = (-map->getOrigin().getX()+posX)/map->getResolution() + ROBOT_WIDTH;
    float newPosY = map->getHeight()-(-map->getOrigin().getY()+posY)/map->getResolution()-ROBOT_WIDTH/2;
    float ori = asin(-oriZ) * 360.0 / PI + 90;

    QPointer<RobotView> rv = robots->getRobotViewByIp(ipAddress);
    if(rv != NULL){
        rv->setPosition(newPosX, newPosY);
        rv->setOrientation(ori);
    } else {
        //qDebug() << "(updateRobot) Could not find a RobotView for the robot at ip" << ipAddress;
    }
}

void MainWindow::launchScan(bool checked){
    qDebug() << "MainWindow::launchScan called" << checked;
    if(selectedRobot != NULL){
        if(checked){
            int ret = openConfirmMessage("Warning, scanning a new map will erase all previously created points, paths and selected home of robots");
            switch(ret){
                case QMessageBox::Cancel :
                    qDebug() << "MainWindow::launchScan clicked cancel";
                    editSelectedRobotWidget->getScanBtn()->setChecked(false);
                    editSelectedRobotWidget->setEnableAll(true);
                break;
                case QMessageBox::Ok :{
                    QString ip = selectedRobot->getRobot()->getIp();
                    qDebug() << "MainWindow::launchScan Trying to connect to : " << ip << ", starting the map worker and thread";

                    if(commandController->sendCommand(selectedRobot->getRobot(), QString("e"))){

                        editSelectedRobotWidget->getScanBtn()->setText("Stop to scan");
                        clearNewMap();
                        editSelectedRobotWidget->getScanBtn()->setEnabled(true);
                        setEnableAll(false, GraphicItemState::SCANNING);
                        scanningRobot = selectedRobot;

                        editSelectedRobotWidget->setEnableAll(false);

                        topLayout->setLabel(TEXT_COLOR_SUCCESS, "Scanning a new map");

                    } else {
                        editSelectedRobotWidget->getScanBtn()->setChecked(false);
                        topLayout->setLabel(TEXT_COLOR_DANGER, "Failed to start to scan a map, please try again");
                    }
                }
                break;
                default:
                    Q_UNREACHABLE();
                    qDebug() << "MainWindow::launchScan should not be here";
                break;
            }
        } else {
            if(commandController->sendCommand(selectedRobot->getRobot(), QString("f"))){
                qDebug() << "MainWindow::launchScan Stopped scanning the map";
                editSelectedRobotWidget->getScanBtn()->setText("Scan a map");

                QPointer<RobotView> _tmpRobot = selectedRobot;
                hideAllWidgets();
                selectedRobot = _tmpRobot;

                editSelectedRobotWidget->setSelectedRobot(selectedRobot);
                editSelectedRobotWidget->setEnableAll(true);
                editSelectedRobotWidget->show();

                setEnableAll(true);
                topLayout->setLabel(TEXT_COLOR_SUCCESS, "Stopped scanning the map");
            } else {
                editSelectedRobotWidget->getScanBtn()->setChecked(true);
                topLayout->setLabel(TEXT_COLOR_DANGER, "Failed to stop the scanning, please try again");
            }
        }
    } else {
        topLayout->setLabelDelay(TEXT_COLOR_DANGER, "You must first click a robot on the map to establish a connection",4000);
        qDebug() << "MainWindow::launchScan You need to select a robot first";
    }
}

void MainWindow::newScanningGoalSlot(double x, double y){
    qDebug() << "MainWindow::newScanningGoalSlot Trying to go to" << x << y;
}

void MainWindow::deletePath(int robotNb){
    qDebug() << "MainWindow::deletepath called on robot :" << robots->getRobotsVector().at(robotNb)->getRobot()->getName();
    QPointer<Robot> robot = robots->getRobotsVector().at(robotNb)->getRobot();
    if(robot->getPath().size() > 0){
        /// if the robot is not playing its path
        if(!robot->isPlayingPath()){
            msgBox.setIcon(QMessageBox::Question);
            int ret = openConfirmMessage("Are you sure you want to delete this path ?");
            switch (ret) {
                case QMessageBox::Ok:
                {
                    /// if the command is succesfully sent to the robot, we apply the change
                    if(commandController->sendCommand(robot, QString("k"))){
                        clearPath(robotNb);
                        topLayout->setLabel(TEXT_COLOR_SUCCESS, "The path of " + robot->getName() + " has been successfully deleted");
                    } else
                        topLayout->setLabel(TEXT_COLOR_DANGER, "Failed to delete the path of " + robot->getName() + ", please try again");
                }
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
                {
                    /// if the command is succesfully sent to the robot, we apply the change
                    QPointer<Robot> robot = robots->getRobotsVector().at(robotNb)->getRobot();
                    if(commandController->sendCommand(robot, QString("m"))){
                        clearPath(robotNb);
                        topLayout->setLabel(TEXT_COLOR_SUCCESS, "The path of " + robot->getName() + " has been successfully deleted");
                    } else {
                        topLayout->setLabel(TEXT_COLOR_DANGER, "Failed to delete the path of " + robot->getName() + ", please try again");
                    }
                }
            }
        }
    } else {
        qDebug() << "This robot has no path";
    }
}

void MainWindow::stopPath(int robotNb){
    qDebug() << "MainWindow::StopPath called";
    QPointer<Robot> robot = robots->getRobotsVector().at(robotNb)->getRobot();
    if(commandController->sendCommand(robot, QString("l"))){
        robot->setPlayingPath(false);
        bottomLayout->getPlayRobotBtnGroup()->button(robotNb)->setIcon(QIcon(":/icons/play.png"));
        bottomLayout->getStopRobotBtnGroup()->button(robotNb)->setEnabled(false);
        topLayout->setLabel(TEXT_COLOR_SUCCESS, "Path stopped");
    } else {
        topLayout->setLabel(TEXT_COLOR_DANGER, "Path failed to be stopped, please try again");
    }
}

void MainWindow::playSelectedRobot(int robotNb){
    QPointer<Robot> robot = robots->getRobotsVector().at(robotNb)->getRobot();
    if(robot->isPlayingPath()){
        qDebug() << "pause path on robot " << robotNb << " : " << robot->getName();
        /// if the command is succesfully sent to the robot, we apply the change
        if(commandController->sendCommand(robot, QString("d"))){
            robot->setPlayingPath(0);
            bottomLayout->getPlayRobotBtnGroup()->button(robotNb)->setIcon(QIcon(":/icons/play.png"));
            bottomLayout->getStopRobotBtnGroup()->button(robotNb)->setEnabled(true);
            topLayout->setLabel(TEXT_COLOR_SUCCESS, "Path stopped");
        } else {
            topLayout->setLabel(TEXT_COLOR_DANGER, "Path failed to be stopped, please try again");
        }
    } else {
        qDebug() << "play path on robot " << robotNb << " : " << robot->getName();

        /// if the command is succesfully sent to the robot, we apply the change
        if(commandController->sendCommand(robot, QString("j"))){
            robot->setPlayingPath(1);
            bottomLayout->getPlayRobotBtnGroup()->button(robotNb)->setIcon(QIcon(":/icons/pause.png"));
            bottomLayout->getStopRobotBtnGroup()->button(robotNb)->setEnabled(true);
            topLayout->setLabel(TEXT_COLOR_SUCCESS, "Path playing");
        } else {
            topLayout->setLabel(TEXT_COLOR_DANGER, "Path failed to start, please try again");
        }
    }
}

void MainWindow::viewPathSelectedRobot(int robotNb, bool checked){
    qDebug() << "MainWindow::viewPathSelectedRobot called" << robotNb << checked;

    if(checked){
        /// in case we were displaying a path from the menu we make sure the eye button as well as the eye icon are unchecked and hidden respectively
        leftMenu->getPathGroupDisplayed()->getActionButtons()->getMapButton()->setChecked(false);
        if(leftMenu->getPathGroupDisplayed()->getPathButtonGroup()->getButtonGroup()->checkedButton())
            leftMenu->getPathGroupDisplayed()->enableButtons(leftMenu->getPathGroupDisplayed()->getPathButtonGroup()->getButtonGroup()->checkedButton());
        displayPathOnMap(false);

        QPointer<Robot> robot = robots->getRobotsVector().at(robotNb)->getRobot();
        qDebug() << "viewPathSelectedRobot called on" << robot->getName();
        bottomLayout->uncheckViewPathSelectedRobot(robotNb);
        pathPainter->setCurrentPath(robot->getPath(), "");
        bottomLayout->updateRobot(robotNb, robots->getRobotsVector().at(robotNb));

    } else {
        if(leftMenu->getDisplaySelectedPoint() && leftMenu->getDisplaySelectedPoint()->isVisible() && leftMenu->getDisplaySelectedPoint()->getPointView()->getPoint()->isPath()){
            leftMenu->getDisplaySelectedPoint()->getPointView()->hide();
            selectedPoint = QSharedPointer<PointView>();
            leftMenu->getDisplaySelectedPoint()->setPointView(selectedPoint, "");
            backEvent();
        }
        emit resetPath();
    }
    if(selectedRobot && selectedRobot->getRobot()->getHome())
        selectedRobot->getRobot()->getHome()->setPixmap(PointView::PixmapType::SELECTED);
}

void MainWindow::setSelectedRobot(QPointer<RobotView> robotView){
    qDebug() << "MainWindow::editselectedrobot robotview" << robotView->getRobot()->getName();
    /// resets the home
    editSelectedRobotWidget->setHome(robotView->getRobot()->getHome());

    /// same thing for path
    if(!robotView->getRobot()->getPathName().compare("")){
        editSelectedRobotWidget->getDeletePathBtn()->hide();
    }
    else {
        editSelectedRobotWidget->getDeletePathBtn()->show();
        qDebug() << robotView->getRobot()->getPathName();
    }

    robots->setSelected(robotView);

    hideAllWidgets();

    selectedRobot = robotView;

    editSelectedRobotWidget->setAssignedPath(selectedRobot->getRobot()->getPathName());
    editSelectedRobotWidget->setGroupPath(selectedRobot->getRobot()->getGroupPathName());

    /// message to explain the user how to assign a path or a home to his robot
    setMessageTop(TEXT_COLOR_NORMAL, "");
    if(selectedRobot->getRobot()->getPath().size() == 0){
        setMessageTop(TEXT_COLOR_INFO, "You can assign a path to your robot by clicking the button labeled \"Assign a path\"");
        if(!selectedRobot->getRobot()->getHome())
            setMessageTop(TEXT_COLOR_INFO, topLayout->getLabel()->text() + "\nYou can assign a home to your robot by clicking the button "
                                                                       "labeled \"Assign a home point\"");
    } else
        if(!selectedRobot->getRobot()->getHome())
            setMessageTop(TEXT_COLOR_INFO, topLayout->getLabel()->text() + "You can assign a home to your robot by clicking the button "
                                                                       "labeled \"Assign a home point\"");

    editSelectedRobotWidget->setSelectedRobot(selectedRobot);
    pathPainter->setPathDeleted(false);

    viewPathSelectedRobot(robots->getRobotId(robotView->getRobot()->getName()), true);
    switchFocus(selectedRobot->getRobot()->getName(), editSelectedRobotWidget, MainWindow::WidgetType::ROBOT);

    /// it was disable by setEnableAll
    leftMenu->getReturnButton()->setEnabled(true);

    emit resetPathCreationWidget();
    pathPainter->setCurrentPath(robotView->getRobot()->getPath(), "");

    showHomes();
    leftMenu->show();
    editSelectedRobotWidget->show();

    /// we uncheck the last robot if such robot exists
    if(bottomLayout->getLastCheckedId() != -1){
        bottomLayout->getViewPathRobotBtnGroup()->button(bottomLayout->getLastCheckedId())->setChecked(false);
        bottomLayout->getRobotBtnGroup()->button(bottomLayout->getLastCheckedId())->setChecked(false);
    }

    if(selectedRobot->getRobot()->getPath().size() > 0)
        bottomLayout->getViewPathRobotBtnGroup()->button(robots->getRobotId(robotView->getRobot()->getName()))->setChecked(true);

    bottomLayout->getRobotBtnGroup()->button(robots->getRobotId(robotView->getRobot()->getName()))->setChecked(true);

    /// if the robot has a home we show the go home button otherwise we hide it
    if(!robotView->getRobot()->getHome())
        editSelectedRobotWidget->getGoHomeBtn()->hide();
    else {
        editSelectedRobotWidget->getGoHomeBtn()->show();
        selectedRobot->getRobot()->getHome()->setPixmap(PointView::PixmapType::SELECTED);
    }
}

void MainWindow::robotBtnEvent(void){
    qDebug() << "robotBtnEvent called";
    leftMenuWidget->hide();
    robotsLeftWidget->show();
    switchFocus("Robots", robotsLeftWidget, MainWindow::WidgetType::ROBOTS);
}

void MainWindow::deletePathSelecRobotBtnEvent(){
    qDebug() << "MainWindow::deletePathSelecRobotBtnEvent called on robot " << selectedRobot->getRobot()->getName();
    deletePath(robots->getRobotId(selectedRobot->getRobot()->getName()));
}

void MainWindow::setSelectedRobotNoParent(QAbstractButton *button){
    qDebug() << "Setselectedrobotnoparent called with id" << bottomLayout->getRobotBtnGroup()->id(button) << ", last id is" << bottomLayout->getLastCheckedId();
    /// displays the robot on the map
    const int robotId = bottomLayout->getRobotBtnGroup()->id(button);
    /// if the button was already checked we uncheck it
    if(bottomLayout->getLastCheckedId() == bottomLayout->getRobotBtnGroup()->id(button)){
        qDebug() << "gotta hide the robot" << button->text();
        /// hides the left menu
        editSelectedRobotWidget->hide();
        leftMenu->hide();
        /// enables the robot buttons (otherwise there is a bug that makes the buttons uncheckable for some reason)
        bottomLayout->uncheckRobots();
        /// resets the last check Id to -1 which means, no robot was selected before me
        bottomLayout->setLastCheckedId(-1);
        /// to change the pixmap of the robot on the map
        robots->deselect();
        /// we hide the path
        bottomLayout->getViewPathRobotBtnGroup()->button(robotId)->setChecked(false);
        selectedRobot = 0;
        points->setPixmapAll(PointView::PixmapType::NORMAL);

    } else if(robotId != -1 ){
        qDebug() << "have to display the robot" << button->text();
        resetFocus();
        /// updates the robot menu on the left to fit this particular robot's information
        setSelectedRobot(robots->getRobotViewByName(button->text()));

        editSelectedRobotWidget->setGroupPath(robots->getRobotViewByName(button->text())->getRobot()->getGroupPathName());
        editSelectedRobotWidget->setHome(robots->getRobotsVector().at(robotId)->getRobot()->getHome());
        editSelectedRobotWidget->setAssignedPath(robots->getRobotViewByName(button->text())->getRobot()->getPathName());
        /// updates the last checked id to the id of the current button / robot
        bottomLayout->setLastCheckedId(robotId);
        editSelectedRobotWidget->show();
        /// show only the home of the selected robot
        showHomes();
        if(selectedRobot->getRobot()->getHome())
            selectedRobot->getRobot()->getHome()->setPixmap(PointView::PixmapType::SELECTED);
    }
}

void MainWindow::setSelectedRobot(QAbstractButton *button){
    Q_UNUSED(button)
    qDebug() << "select a robot in robot group ";

    if(robotsLeftWidget->getLastCheckedId() != robotsLeftWidget->getBtnGroup()->getBtnGroup()->id(button)){
        robotsLeftWidget->setLastCheckedId(robotsLeftWidget->getBtnGroup()->getBtnGroup()->id(button));
        robotsLeftWidget->getActionButtons()->getEditButton()->setEnabled(true);
        robotsLeftWidget->getActionButtons()->getMapButton()->setEnabled(true);
        QPointer<RobotView> mySelectedRobot = robots->getRobotViewByName(static_cast<CustomPushButton *> (robotsLeftWidget->getBtnGroup()->getBtnGroup()->checkedButton())->text());
        editSelectedRobotWidget->setGroupPath(mySelectedRobot->getRobot()->getGroupPathName());
        editSelectedRobotWidget->setAssignedPath(mySelectedRobot->getRobot()->getPathName());

        const int robotId = robotsLeftWidget->getBtnGroup()->getBtnGroup()->id(button);
        robotsLeftWidget->getActionButtons()->getMapButton()->setChecked(mySelectedRobot->isVisible());
        /// to show the selected robot with a different color
        robots->deselect();
        robots->getRobotsVector().at(robotId)->setSelected(true);
        /// to select the robot in the bottom layout accordingly
        bottomLayout->uncheckRobots();
        bottomLayout->getRobotBtnGroup()->button(robotId)->setChecked(true);
        bottomLayout->setLastCheckedId(robotId);
    } else {
        robotsLeftWidget->getBtnGroup()->uncheck();
        robotsLeftWidget->getActionButtons()->getMapButton()->setChecked(false);
        robotsLeftWidget->setLastCheckedId(-1);
        robots->deselect();
        bottomLayout->uncheckRobots();
        bottomLayout->setLastCheckedId(-1);
        robotsLeftWidget->getActionButtons()->getEditButton()->setEnabled(false);
        robotsLeftWidget->getActionButtons()->getMapButton()->setEnabled(false);
    }
}

void MainWindow::selectViewRobot(){
    qDebug() << "select view robot  " <<robotsLeftWidget->getSelectedRobotName();
    setSelectedRobot(robots->getRobotViewByName(robotsLeftWidget->getSelectedRobotName()));
}

void MainWindow::setSelectedRobotFromPointSlot(QString robotName){
    qDebug() << "MainWindow::setSelectedRobotFromPointSlot called :" << robotName;
    QPointer<RobotView> rv = robots->getRobotViewByName(robotName);
    if(rv != NULL)
        setSelectedRobot(rv);
    else
        qDebug() << "MainWindow::setSelectedRobotFromPointSlot : Error could not find the robot named :" << robotName;
}

void MainWindow::backRobotBtnEvent(){
    qDebug() << "backRobotBtnEvent called";
    robotsLeftWidget->hide();
    leftMenuWidget->show();
}

void MainWindow::editRobotBtnEvent(){
    qDebug() << "editRobotBtnEvent called";
    /// hides a previously shown stand-alone path
    emit resetPath();
    setSelectedRobot(robots->getRobotViewByName(static_cast<CustomPushButton*> (robotsLeftWidget->getBtnGroup()->getBtnGroup()->checkedButton())->text()));
}

void MainWindow::checkRobotBtnEventMenu(){
    qDebug() << "checkRobotBtnEventMenu called";
    QString name = robotsLeftWidget->getBtnGroup()->getBtnGroup()->checkedButton()->text();

    checkRobotBtnEvent(name);
}

void MainWindow::checkRobotBtnEvent(QString name){
    qDebug() << "checkRobotBtnEvent called" << name;


   QPointer<RobotView> robotView =  robots->getRobotViewByName(name);
     robotView->display(!robotView->isVisible());
}

void MainWindow::cancelEditSelecRobotBtnEvent(){
    qDebug() << "cancelEditSelecRobotBtnEvent called";
    /// resets the name
    if(editSelectedRobotWidget->getHome())
        editSelectedRobotWidget->getHomeLabel()->setText("Home : " + editSelectedRobotWidget->getHome()->getPoint()->getName());
    else
        editSelectedRobotWidget->getHomeLabel()->setText("Home : ");
    /// if a home has been edited we reset it to its old value which might be a null pointer
    if(editSelectedRobotWidget->getHome()){
        qDebug() << "my home is" << editSelectedRobotWidget->getHome()->getPoint()->getName();
        editSelectedRobotWidget->getHome()->getPoint()->setHome(Point::PERM);
        editSelectedRobotWidget->getHome()->setPixmap(PointView::PixmapType::NORMAL);
    }

    if(editSelectedRobotWidget->getHome())
        editSelectedRobotWidget->setHome(editSelectedRobotWidget->getHome());
    else
        editSelectedRobotWidget->setHome(static_cast<QSharedPointer<PointView>> (0));

    editSelectedRobotWidget->updateHomeMenu();
    /// if the path has been changed, reset the path
    emit resetPathCreationWidget();
    displayAssignedPath(selectedRobot->getRobot()->getGroupPathName(), selectedRobot->getRobot()->getPathName());

    editSelectedRobotWidget->setGroupPath(selectedRobot->getRobot()->getGroupPathName());
    editSelectedRobotWidget->setAssignedPath(selectedRobot->getRobot()->getPathName());
    editSelectedRobotWidget->setHome(editSelectedRobotWidget->getHome());
    editSelectedRobotWidget->updatePathsMenu();

    /// the user may have done a mistake regarding which parameter he wants to modify but he still wants to modify something
    /// so it might be better to stay on the same page
    //backEvent();
    editSelectedRobotWidget->setPathChanged(false);
    points->getTmpPointView()->setPixmap(PointView::PixmapType::MID);
    points->getTmpPointView()->hide();

    leftMenu->getReturnButton()->setEnabled(true);
    leftMenu->getReturnButton()->setToolTip("");

    setEnableAll(true);

    //setTemporaryMessageTop(TEXT_COLOR_INFO, "You have cancelled all the modifications made to the robot " + selectedRobot->getRobot()->getName(), 2500);
}

/// TO DELETE
void MainWindow::robotSavedEvent(){
    qDebug() << "MainWindow::robotSavedEvent called";
/*
    bool isOK = false;
    int change = 0;

    if(editSelectedRobotWidget->getPathChanged()){
        change++;
        isOK = true;
    }

    /// if we changed the name
    if(selectedRobot->getRobot()->getName().compare(editSelectedRobotWidget->getRobotInfoDialog()->getNameEdit()->text()) != 0){
        qDebug() << "MainWindow::robotSavedEvent Name has been modified";
        if(selectedRobot->getRobot()->commandController->sendCommand(QString("a \"") + editSelectedRobotWidget->getRobotInfoDialog()->getNameEdit()->text() + "\"")){
            QString answer = selectedRobot->getRobot()->waitAnswer();
            QStringList answerList = answer.split(QRegExp("[ ]"), QString::SkipEmptyParts);
            if(answerList.size() > 1){
                QString cmd = answerList.at(0);
                bool success = (answerList.at(1).compare("done") == 0);
                if((cmd.compare("a") == 0 && success) || answerList.at(0).compare("1") == 0){
                    isOK = true;
                    change++;

                    QMap<QString, QString> tmp = robots->getRobotsNameMap();
                    tmp[selectedRobot->getRobot()->getIp()] = editSelectedRobotWidget->getRobotInfoDialog()->getNameEdit()->text();
                    emit changeCmdThreadRobotName(editSelectedRobotWidget->getRobotInfoDialog()->getNameEdit()->text());
                    robots->setRobotsNameMap(tmp);

                    QFile fileWrite(QString(GOBOT_PATH) + QString(ROBOTS_NAME_FILE));
                    fileWrite.resize(0);
                    fileWrite.open(QIODevice::WriteOnly);
                    QDataStream out(&fileWrite);
                    out << robots->getRobotsNameMap();
                    fileWrite.close();
                    qDebug() << "MainWindow::robotSavedEvent RobotsNameMap updated" << robots->getRobotsNameMap();
                } else {
                    isOK = false;
                    setMessageTop(TEXT_COLOR_DANGER, "Failed to edit the name of the robot");
                }
            }
        }
    }

    /// if we changed the wifi
    if (editSelectedRobotWidget->getRobotInfoDialog()->getPasswordEdit()->text() != "......"){
        qDebug() << "MainWindow::robotSavedEvent Wifi has been modified";
        if(selectedRobot->getRobot()->commandController->sendCommand(QString("b \"")
                  + editSelectedRobotWidget->getRobotInfoDialog()->getNameEdit()->text() + "\" \""
                  + editSelectedRobotWidget->getRobotInfoDialog()->getPasswordEdit()->text() + "\"")){

            QString answer2 = selectedRobot->getRobot()->waitAnswer();
            QStringList answerList2 = answer2.split(QRegExp("[ ]"), QString::SkipEmptyParts);
            if(answerList2.size() > 1){
                QString cmd2 = answerList2.at(0);
                bool success2 = (answerList2.at(1).compare("done") == 0);
                if((cmd2.compare("b") == 0 && success2) || answerList2.at(0).compare("1") == 0){
                    isOK = true;
                    change++;
                } else {
                    isOK = false;
                    setMessageTop(TEXT_COLOR_DANGER, "Failed to edit the wifi");
                }
            }
        }
    }

    /// if we changed the home
    QSharedPointer<PointView> pointView = editSelectedRobotWidget->getHome();
    if(pointView != NULL && (selectedRobot->getRobot()->getHome() == NULL || !(*(pointView->getPoint()) == *(selectedRobot->getRobot()->getHome()->getPoint())))){
        qDebug() << "MainWindow::robotSavedEvent Home has been modified";
        bool done = false;

        if(pointView->getPoint()->isTemporary()){
            if(selectedRobot->getRobot()->commandController->sendCommand(QString("n \"") + QString::number(pointView->getPoint()->getPosition().getX()) + "\" \""
                                                              + QString::number(pointView->getPoint()->getPosition().getY()) + "\"")){
                QString answerHome = selectedRobot->getRobot()->waitAnswer();
                QStringList answerList2 = answerHome.split(QRegExp("[ ]"), QString::SkipEmptyParts);
                if(answerList2.size() > 1){
                    QString cmd2 = answerList2.at(0);
                    bool success2 = (answerList2.at(1).compare("done") == 0);
                    if((cmd2.compare("n") == 0 && success2) || answerList2.at(0).compare("1") == 0){

                        qDebug() << "MainWindow::robotSavedEvent Tmp point";
                        pointView->getPoint()->setHome(Point::PointType::HOME);
                        pointView->getPoint()->setRobotName(selectedRobot->getRobot()->getName());

                        points->addPoint(NO_GROUP_NAME, points->getGroups()->value(TMP_GROUP_NAME)->takeFirst());
                        points->addTmpPoint();

                        XMLParser parserPoints(QDir::currentPath());
                        parserPoints.save(*points);
                        done = true;
                        setMessageTop(TEXT_COLOR_SUCCESS, selectedRobot->getRobot()->getName() + " successfully saved its home point");
                    } else
                        setMessageTop(TEXT_COLOR_DANGER, selectedRobot->getRobot()->getName() + " failed to save its home point, please try again");
                }
            }
        } else {
            qDebug() << "MainWindow::robotSavedEvent Permanent point";
            if(pointView->getPoint()->setHome(Point::PointType::HOME)){
                if(selectedRobot->getRobot()->commandController->sendCommand(QString("n \"") + QString::number(pointView->getPoint()->getPosition().getX()) + "\" \""
                                                                  + QString::number(pointView->getPoint()->getPosition().getY()) + "\"")){
                    QString answerHome = selectedRobot->getRobot()->waitAnswer();
                    QStringList answerList2 = answerHome.split(QRegExp("[ ]"), QString::SkipEmptyParts);
                    if(answerList2.size() > 1){
                        QString cmd2 = answerList2.at(0);
                        bool success2 = (answerList2.at(1).compare("done") == 0);
                        if((cmd2.compare("n") == 0 && success2) || answerList2.at(0).compare("1") == 0){
                            XMLParser parserPoints(QDir::currentPath());
                            parserPoints.save(*points);
                            done = true;

                            setMessageTop(TEXT_COLOR_SUCCESS, selectedRobot->getRobot()->getName() + " successfully saved its home point");
                        } else
                            setMessageTop(TEXT_COLOR_DANGER, selectedRobot->getRobot()->getName() + " failed to save its home point, please try again");
                    }
                }
            } else
                setMessageTop(TEXT_COLOR_DANGER, "Sorry, this point is already a home\nPlease select another");
        }
        pointsLeftWidget->updateGroupButtonGroup();


        if(done){
            if(selectedRobot->getRobot()->getHome() != NULL)
                selectedRobot->getRobot()->getHome()->getPoint()->setHome(Point::PointType::PERM);
            selectedRobot->getRobot()->setHome(editSelectedRobotWidget->getHome());
        }

        isOK = true;
        change++;
    }

    /// if we changed the path
    if(editSelectedRobotWidget->getPathChanged()){
        qDebug() << "MainWindow::robotSavedEvent path changed";
        QPointer<Robot> robot = selectedRobot->getRobot();
        QString pathStr = "";
        /// prepares the cmd to send to the robot
        qDebug() << "MainWindow::robotSavedEvent" << pathPainter->getCurrentPath().size();
        for(int i = 0; i < pathPainter->getCurrentPath().size(); i++){
            QSharedPointer<PathPoint> pathPoint = pathPainter->getCurrentPath().at(i);
            float oldPosX = pathPoint->getPoint().getPosition().getX();
            float oldPosY = pathPoint->getPoint().getPosition().getY();

            float newPosX = (oldPosX - ROBOT_WIDTH) * map->getResolution() + map->getOrigin().getX();
            float newPosY = (-oldPosY + map->getHeight() - ROBOT_WIDTH/2) * map->getResolution() + map->getOrigin().getY();
            int waitTime = -1;
            if(pathPoint->getAction() == PathPoint::WAIT){
                waitTime = pathPoint->getWaitTime();
            }
            pathStr += + "\"" + QString::number(newPosX) + "\" \"" + QString::number(newPosY) + "\" \"" + QString::number(waitTime)+ "\" ";
        }

        /// if the command is succesfully sent to the robot, we apply the change
        // check if pathStr empty, means deleting the path => cmd send without parem => check robotFiles/command.cpp
        if(robot->commandController->sendCommand(QString("i ") + pathStr)){
            QString answer = robot->waitAnswer();
            QStringList answerList = answer.split(QRegExp("[ ]"), QString::SkipEmptyParts);
            if(answerList.size() > 1){
                QString cmd = answerList.at(0);
                bool success = (answerList.at(1).compare("done") == 0);
                if((cmd.compare("i") == 0 && success) || answerList.at(0).compare("1") == 0){
                    /// we update the path on the application side by serializing the path

                    qDebug() << "MainWindow::robotSavedEvent Path saved for robot" << robot->getIp();
                } else {
                    qDebug() << "MainWindow::robotSavedEvent Path failed to be saved, please try again";
                }
            }
        }
    }

    /// finally we edit
    if(editSelectedRobotWidget->isVisible()){
        //if(change > 0){
            //if (isOK){
                qDebug() << "can update";
                if(editSelectedRobotWidget->getPathChanged()){
                    selectedRobot->getRobot()->setPath(pathPainter->getCurrentPath());
                    selectedRobot->getRobot()->setGroupPathName(editSelectedRobotWidget->getGroupPathName());
                    selectedRobot->getRobot()->setPathName(editSelectedRobotWidget->getPathName());
                    int id = robots->getRobotId(selectedRobot->getRobot()->getName());
                    bottomLayout->updateRobot(id, selectedRobot);
                    if(pathPainter->getCurrentPath().size() > 0){
                        bottomLayout->getViewPathRobotBtnGroup()->button(id)->setChecked(true);
                        viewPathSelectedRobot(id, true);
                    }
                }

                /// resets the old home so that there is only one home per robot
                QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
                while (i.hasNext()) {
                    i.next();
                    if(i.value() && i.key().compare(PATH_GROUP_NAME) != 0 && i.key().compare(TMP_GROUP_NAME)){
                        for(int j = 0; j < i.value()->count(); j++){
                            if(!i.value()->at(j)->getPoint()->getRobotName().compare(selectedRobot->getRobot()->getName())){
                                /// to make the old home a normal point again
                                i.value()->at(j)->getPoint()->setHome(Point::PERM);
                                i.value()->at(j)->setPixmap(PointView::PixmapType::NORMAL);
                            }
                        }
                    }
                }

                /// to reset the color of the home
                if(editSelectedRobotWidget->getHome()){
                    qDebug() << "resetting the color of my home" << editSelectedRobotWidget->getHome()->getPoint()->getName();
                    editSelectedRobotWidget->getHome()->setPixmap(PointView::PixmapType::NORMAL);
                }

                editSelectedRobotWidget->setHome(editSelectedRobotWidget->getHome());

                editSelectedRobotWidget->updateHomeMenu();

                if(pointView){
                    pointView->getPoint()->setHome(Point::PointType::HOME);
                    pointView->getPoint()->setRobotName(selectedRobot->getRobot()->getName());
                    selectedRobot->getRobot()->setHome(pointView);
                } else
                    qDebug() << "no pointview";


                pathPainter->clearOldPath();
                editSelectedRobotWidget->setPathChanged(false);

                //backEvent();
                //leftMenu->getReturnButton()->setEnabled(true);
                leftMenu->getReturnButton()->setToolTip("");

                editSelectedRobotWidget->editName();

                setEnableAll(true);
                robotsLeftWidget->updateRobots(robots);
                bottomLayout->updateRobot(robots->getRobotId(selectedRobot->getRobot()->getName()), selectedRobot);

                editSelectedRobotWidget->setEditing(false);

                setTemporaryMessageTop(TEXT_COLOR_SUCCESS, "The information of the robot " + selectedRobot->getRobot()->getName() + " have been successfully updated", 2500);
           // }

        } else {
            setMessageTop(TEXT_COLOR_INFO, "Nothing has been modified because this point was already the home point of the robot " + selectedRobot->getRobot()->getName());
            qDebug() << "Nothing has been modified because this point was already the home point of the robot " + selectedRobot->getRobot()->getName();
        }
    }*/
}


void MainWindow::saveRobotModifications(){
    qDebug() << "MainWindow::saveRobotModifications called";
    bool nameChanged = false;
    bool wifiChanged = false;
    CustomRobotDialog* robotDialog = editSelectedRobotWidget->getRobotInfoDialog();
    /// we check if the name has been changed
    if(!robotDialog->getNameEdit()->text().isEmpty() &&
            robotDialog->getNameEdit()->text().simplified().compare(selectedRobot->getRobot()->getName(), Qt::CaseSensitive)){
        if(changeRobotName(robotDialog->getNameEdit()->text().simplified())){
            nameChanged = true;
        } else {
            setMessageTop(TEXT_COLOR_DANGER, "Failed to edit the name of the robot, please try again");
            return;
        }
    }

    editSelectedRobotWidget->getRobotInfoDialog()->close();

    /// we check if the SSID or the password have changed
    if((!robotDialog->getPasswordEdit()->text().isEmpty() &&
            robotDialog->getPasswordEdit()->text() != "......") ||
            (!robotDialog->getSSIDEdit()->text().isEmpty() &&
            robotDialog->getSSIDEdit()->text().compare(selectedRobot->getRobot()->getWifi(), Qt::CaseSensitive))){
        changeRobotWifi(robotDialog->getSSIDEdit()->text().simplified(), robotDialog->getPasswordEdit()->text().simplified());
        wifiChanged = true;
    }


    if(nameChanged && wifiChanged){
        setMessageTop(TEXT_COLOR_SUCCESS, "You have successfully updated the name and the wifi of " + robotDialog->getNameEdit()->text().simplified());
    } else {
        if(nameChanged)
            setMessageTop(TEXT_COLOR_SUCCESS, "You have successfully updated the name of " + robotDialog->getNameEdit()->text().simplified());
        else if(wifiChanged)
            setMessageTop(TEXT_COLOR_SUCCESS, "You have successfully updated the wifi of " + robotDialog->getNameEdit()->text().simplified());
    }

    robotDialog->getNameEdit()->setText(selectedRobot->getRobot()->getName());
    robotDialog->getSSIDEdit()->setText(selectedRobot->getRobot()->getWifi());
    robotDialog->getPasswordEdit()->setText("......");
}

bool MainWindow::changeRobotName(QString name){
    qDebug() << "MainWindow::changeRobotName called";

    if(commandController->sendCommand(selectedRobot->getRobot(), QString("a \"") + name + "\"")){
        /// updates the name of the file which stores the path of the robot
        QFile robotPathFile(QDir::currentPath() + QDir::separator() + "robots_paths" + QDir::separator() + selectedRobot->getRobot()->getName() + "_path");
        if(robotPathFile.exists())
            robotPathFile.rename(QDir::currentPath() + QDir::separator() + "robots_paths" + QDir::separator() + name + "_path");

        /// updates the name of the file which stores the home of the robot
        QFile robotHomeFile(QDir::currentPath() + QDir::separator() + "robots_homes" + QDir::separator() + selectedRobot->getRobot()->getName());
        if(robotHomeFile.exists())
            robotHomeFile.rename(QDir::currentPath() + QDir::separator() + "robots_homes" + QDir::separator() + name);

        QMap<QString, QString> tmp = robots->getRobotsNameMap();
        tmp[selectedRobot->getRobot()->getIp()] = name;
        selectedRobot->getRobot()->setName(name);
        robots->setRobotsNameMap(tmp);

        emit changeCmdThreadRobotName(name);
        QFile fileWrite(QDir::currentPath() + QDir::separator() + QString(ROBOTS_NAME_FILE));
        fileWrite.resize(0);
        fileWrite.open(QIODevice::WriteOnly);
        QDataStream out(&fileWrite);
        out << robots->getRobotsNameMap();
        fileWrite.close();

        qDebug() << "MainWindow::robotSavedEvent RobotsNameMap updated" << robots->getRobotsNameMap();
        bottomLayout->updateRobot(robots->getRobotId(selectedRobot->getRobot()->getName()), selectedRobot);
        editSelectedRobotWidget->getNameLabel()->setText(name);
        editSelectedRobotWidget->getRobotInfoDialog()->getNameEdit()->setText(name);

        robotsLeftWidget->updateRobots(robots);
        return true;
    }
    return false;
}

void MainWindow::changeRobotWifi(QString ssid, QString password){
    qDebug() << "MainWindow::changeRobotWifi called";
    commandController->sendCommand(selectedRobot->getRobot(), QString("b \"") + ssid + "\" \"" + password + "\"");
    editSelectedRobotWidget->getWifiNameLabel()->setText(ssid);
    selectedRobot->getRobot()->setWifi(ssid);
}


void MainWindow::editTmpPathPointSlot(int id, QString name, double x, double y){
    qDebug() << "MainWindow::editTmpPathPointSlot called : " << id << name << x << y;
    editedPointView = points->getGroups()->value(PATH_GROUP_NAME)->at(id);
    setGraphicItemsState(GraphicItemState::NO_EVENT);
    if(editedPointView == NULL)
        qDebug() << "MainWindow::editTmpPathPointSlot Error : No pointview found to edit";
    leftMenu->setEnableReturnCloseButtons(false);
    int nbWidget(-1);

    setMessageTop(TEXT_COLOR_INFO, "Drag the selected point or click the map and click \"Save changes\" to modify the path");
    nbWidget = pathPainter->nbUsedPointView(name, x ,y);
    mapPixmapItem->setState(GraphicItemState::EDITING_PATH);
    editedPointView->setState(GraphicItemState::EDITING_PATH);

    qDebug() << "MainWindow::editTmpPathPointSlot number of widget with the same pointView : " << nbWidget;
    /// if 2 path points have the same pointView, we need to create a copy to only
    /// move one of the two path points
    if(nbWidget > 1){
        editedPointView = points->createPoint(
                    editedPointView->getPoint()->getName(),
                    editedPointView->getPoint()->getPosition().getX(),
                    editedPointView->getPoint()->getPosition().getY(),
                    true, Point::PointType::PATH);
        points->getGroups()->value(PATH_GROUP_NAME)->remove(id);
        points->insertPoint(PATH_GROUP_NAME, id, editedPointView);
    }

    editedPointView->setState(GraphicItemState::EDITING_PATH);

    editedPointView->setFlag(QGraphicsItem::ItemIsMovable);
    /// set by hand in order to keep the colors consistent while editing the point and after
    editedPointView->setPixmap(PointView::PixmapType::SELECTED);

}

void MainWindow::savePathSlot(){
    qDebug() << "MainWindow::savePath called";
    QString pathName("");
    /// we hide the points that we displayed for the edition of the path
    for(int i = 0; i < pointViewsToDisplay.size(); i++){
        bool hidePointView(true);
        for(int j = 0; j < pathPainter->getCurrentPath().size(); j++){
            if(pathPainter->getCurrentPath().at(j)->getPoint() == *(pointViewsToDisplay.at(i)->getPoint())){
                hidePointView = false;
                break;
            }
        }
        if(hidePointView)
            pointViewsToDisplay.at(i)->hide();
    }
    pointViewsToDisplay.clear();

    pathPainter->setPathDeleted(false);
    pathPainter->setOldPath(pathPainter->getCurrentPath());

    /// gotta update the model and serialize the paths
    const QString groupName = pathCreationWidget->getCurrentGroupName();
    pathName = pathCreationWidget->getNameEdit()->text().simplified();

    /// if the path existed before we destroy it and reconstruct it
    if(pathCreationWidget->getCurrentPathName().compare("") != 0)
        paths->deletePath(groupName, pathName);

    paths->createPath(groupName, pathName);
    for(int i = 0; i < pathPainter->getCurrentPath().size(); i++)
        paths->addPathPoint(groupName, pathName, pathPainter->getCurrentPath().at(i));

    /// updates the visible path
    pathPainter->setVisiblePath(pathName);

    /// resets the menu so that it reflects the creation of this new path
    leftMenu->getPathGroupDisplayed()->setPathsGroup(pathCreationWidget->getCurrentGroupName());

    /// add this path to the file
    serializePaths(QDir::currentPath() + QDir::separator() + "paths.dat");

    leftMenu->getDisplaySelectedPath()->updatePath(pathCreationWidget->getCurrentGroupName(),
                                                   pathCreationWidget->getNameEdit()->text().simplified(),
                                                   pathPainter->getCurrentPath(),
                                                   pathPainter->getVisiblePath());

    leftMenu->setEnableReturnCloseButtons(true);

    emit updatePathPainter(true);
    backEvent();
}

void MainWindow::addPointPathSlot(QString name, double x, double y, GraphicItemState){
    qDebug() << "addPathPoint called on point via * point" << x << y;
    /// Relay to pathPainter::addPathPointSlot()
    emit addPathPoint(name, x, y);
}

void MainWindow::saveEditPathPointSlot(){
    qDebug() << "MainWindow::saveEditPathPointSlot called";

    setEnableAll(false, GraphicItemState::CREATING_PATH, true);

    pathPainter->updateCurrentPath();

    editedPointView->setFlag(QGraphicsItem::ItemIsMovable, false);
    editedPointView = QSharedPointer<PointView>();
    emit updatePathPainter(true);
}

void MainWindow::cancelEditPathPointSlot(){
    qDebug() << "MainWindow::cancelEditPathPointSlot called";

    setEnableAll(false, GraphicItemState::CREATING_PATH, true);

    Position pos;
    int id(-1);

    id = pathCreationWidget->getPathPointList()->row(pathCreationWidget->getPathPointList()->currentItem());
    pos = pathPainter->getCurrentPath().at(id)->getPoint().getPosition();

    editedPointView->setPos(pos.getX(), pos.getY());

    emit updatePathPainter(true);

    editedPointView->setFlag(QGraphicsItem::ItemIsMovable, false);
    editedPointView = QSharedPointer<PointView>();
}

void MainWindow::updatePathPainterPointViewSlot(){
    /// Relay to pathPainter::updatePathPainterSlot()
    emit updatePathPainterPointView();
}

void MainWindow::showHome(){
    //qDebug() << "MainWindow::showHome called" << (selectedRobot->getRobot()->getHome()==NULL);

    points->setPixmapAll(PointView::PixmapType::NORMAL);

    if(selectedRobot->getRobot() && selectedRobot->getRobot()->getHome() != NULL){
        QSharedPointer<PointView> pointView = selectedRobot->getRobot()->getHome();
        if(pointView->isVisible()){
            qDebug() << "home is visible";
            pointView->setWasShown(true);
        }else{
            qDebug() << "home is not visible";
            pointView->setWasShown(false);
        }

        pointView->show();
    }

    if(!editSelectedRobotWidget->isEditing()){
        bottomLayout->uncheckAll();
        if(pathPainter->getOldPath().size() > 0){
            editSelectedRobotWidget->setPath(pathPainter->getOldPath());

            pathPainter->clearOldPath();
        } else {
            QPointer<RobotView> robotView =  robots->getRobotViewByName(selectedRobot->getRobot()->getName());
            /// If the robot has a path, we display it, otherwise we show the button to add the path
            if(robotView->getRobot()->getPath().size() > 0){
                bottomLayout->getViewPathRobotBtnGroup()->button(robots->getRobotId(selectedRobot->getRobot()->getName()))->setChecked(true);

                editSelectedRobotWidget->getPathWidget()->setPath(robotView->getRobot()->getPath());
                editSelectedRobotWidget->getPathWidget()->show();

                editSelectedRobotWidget->setPath(robotView->getRobot()->getPath());
            } else {
                qDebug() << "MainWindow::showHome I don't have a path !";
                editSelectedRobotWidget->getPathWidget()->hide();

                editSelectedRobotWidget->clearPath();
            }
        }
    }
}

void MainWindow::showEditHome(){
    showHome();
    if(!editSelectedRobotWidget->isEditing()){
        editSelectedRobotWidget->setEditing(true);
    }
}

void MainWindow::clearPath(const int robotNb){
    qDebug() << "MainWindow::clearPath called";

    emit resetPath();
    emit resetPathCreationWidget();
    robots->getRobotsVector().at(robotNb)->getRobot()->clearPath();
    bottomLayout->uncheckAll();
    pathPainter->setPathDeleted(true);

    /// to uncheck the previously checked path
    robots->getRobotsVector().at(robotNb)->getRobot()->setPathName("");
    robots->getRobotsVector().at(robotNb)->getRobot()->setGroupPathName("");

    /// serializes the new path (which is actually an empty path)
    QFile fileInfo(QDir::currentPath() + QDir::separator() + "robots_paths" + QDir::separator()
                   + robots->getRobotsVector().at(robotNb)->getRobot()->getName() + "_path");
    if(fileInfo.open(QIODevice::ReadWrite)){
        fileInfo.resize(0);
        QTextStream out(&fileInfo);
        QString currentDateTime = QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss");
        out << currentDateTime;
        out << "%" << " " << "%" << " ";
        qDebug() << "date now is" << currentDateTime;
        fileInfo.close();
    }

    if(robots->getRobotsVector().at(robotNb)->getRobot()->isPlayingPath()){
        qDebug() << "MainWindow::clearPath pause path on robot before supp " << robotNb << " : " << robots->getRobotsVector().at(robotNb)->getRobot()->getName();
        robots->getRobotsVector().at(robotNb)->getRobot()->setPlayingPath(0);
        bottomLayout->getPlayRobotBtnGroup()->button(robotNb)->setIcon(QIcon(":/icons/play.png"));
    }

    if(editSelectedRobotWidget->isVisible()){
        editSelectedRobotWidget->setSelectedRobot(robots->getRobotsVector().at(robotNb));
        editSelectedRobotWidget->setPathChanged(true);
        editSelectedRobotWidget->clearPath();
        editSelectedRobotWidget->updatePathsMenu();
        editSelectedRobotWidget->getPathWidget()->hide();
    }

    bottomLayout->updateRobot(robotNb, robots->getRobotsVector().at(robotNb));
}

void MainWindow::showAllHomes(void){
    qDebug() << "MainWindow::showAllHomes called after editselectrobot went hidden";
    /// shows the home of each robot
    selectedRobot = 0;
    pathPainter->setCurrentPath(pathPainter->getCurrentPath(), pathPainter->getVisiblePath());
    bottomLayout->uncheckRobots();
}

void MainWindow::robotIsAliveSlot(QString hostname, QString ip, QString mapId, QString mapDate, QString ssid, int stage){
    QRegExp rx("[0-9]{1,3}\\.[0-9]{1,3}\\.[0-9]{1,3}\\.[0-9]{1,3}");
    rx.indexIn(ip);
    ip = rx.cap(0);
    QPointer<RobotView> rv = robots->getRobotViewByIp(ip);

    if(rv != NULL){
        qDebug() << "Robot" << hostname << "at ip" << ip << "is still alive and has the map id :" << mapId;
        rv->getRobot()->ping();
        /// TODO see for changes (battery)

    } else {
        qDebug() << "Robot" << hostname << "at ip" << ip << "just connected and has the map id :" << mapId;

        QPointer<Robot> robot = QPointer<Robot>(new Robot(this, paths, hostname, ip));
        robot->setWifi(ssid);
        rv = QPointer<RobotView>(new RobotView(robot, mapPixmapItem));
        connect(rv, SIGNAL(setSelectedSignal(QPointer<RobotView>)), this, SLOT(setSelectedRobot(QPointer<RobotView>)));
        rv->setPosition(robots->getRobotsVector().count()*100+100, robots->getRobotsVector().count()*100+100);
        rv->setParentItem(mapPixmapItem);
        robots->add(rv);
        robot->launchWorkers(this);
        bottomLayout->addRobot(rv);
        robotsLeftWidget->updateRobots(robots);

        /// Check if connection by usb
        if(ip.endsWith(".7.1") || ip.endsWith(".7.2") || ip.endsWith(".7.3")){
            hideAllWidgets();
            selectedRobot = rv;
            switchFocus(hostname, editSelectedRobotWidget, MainWindow::WidgetType::ROBOT);
            editSelectedRobotWidget->setSelectedRobot(selectedRobot, true);
            editSelectedRobotWidget->show();
            leftMenu->show();
            setEnableAll(false, GraphicItemState::NO_EVENT);
        } else {
            QMap<QString, QString> tmp = robots->getRobotsNameMap();
            tmp[ip] = hostname;
            robots->setRobotsNameMap(tmp);
            QFile fileWrite(QDir::currentPath() + QDir::separator() + QString(ROBOTS_NAME_FILE));
            fileWrite.resize(0);
            fileWrite.open(QIODevice::WriteOnly);
            QDataStream out(&fileWrite);
            out << robots->getRobotsNameMap();
            fileWrite.close();
            qDebug() << "RobotsNameMap updated" << robots->getRobotsNameMap();
        }
    }

    /// Check if the robot has the current map
    //qDebug() << "Robot" << hostname << "comparing ids" << mapId << "and" << map->getMapId().toString();
    if(mapId.compare(map->getMapId().toString()) == 0){
        qDebug() << "Robot" << hostname << "has the current map";
    } else {
        // TODO Send or receive the map depending on settings
        QDateTime mapDateTime = QDateTime::fromString(mapDate, "yyyy-MM-dd-hh-mm-ss");
        //qDebug() << "Robot" << hostname << "comparing date" << mapDateTime << "and" << map->getDateTime();
        if(mapDateTime <= map->getDateTime()){
            qDebug() << "Robot" << hostname << "has an older map";
        } else {
            qDebug() << "Robot" << hostname << "has a newer map";
        }
    }

    int robotId = robots->getRobotId(rv->getRobot()->getName());

    /// updates the text in the bottom layout to make the stage appear
    if(rv->getLastStage() != stage){
        rv->setLastStage(stage);
        bottomLayout->updateStageRobot(robotId, rv, stage);
    }

    /// Check the current stage of the robot
    if(rv->getRobot()->isPlayingPath() && rv->getRobot()->getPath().size() == stage){
        setMessageTop(TEXT_COLOR_SUCCESS, "The robot " + rv->getRobot()->getName() + " has successfully reached its destination");
        bottomLayout->getPlayRobotBtnGroup()->button(robotId)->setIcon(QIcon(":/icons/play.png"));
        bottomLayout->getStopRobotBtnGroup()->button(robotId)->setEnabled(false);
    }
}

void MainWindow::robotIsDeadSlot(QString hostname,QString ip){
    qDebug() << "Robot" << hostname << "at ip" << ip << "... He is dead, Jim!!";
    setMessageTop(TEXT_COLOR_DANGER, QString("Robot " + hostname + " at ip " + ip +" disconnected."));

    qDebug() << "Robots IPs : ";
    for(int i = 0; i < robots->getRobotsVector().size(); i++){
        qDebug() << robots->getRobotsVector().at(i)->getRobot()->getIp();
    }

    QPointer<RobotView> rv = robots->getRobotViewByIp(ip);
    int id = robots->getRobotId(hostname);

    if(rv != NULL && rv->getRobot() != NULL){

        /// if the robot had a home, make the point a normal point
        if(rv->getRobot()->getHome() != NULL)
            rv->getRobot()->getHome()->getPoint()->setHome(Point::PointType::PERM);

        /// if selected => if one of this robot related menu is open
        if(selectedRobot && selectedRobot->getRobot()->getIp().compare(ip) == 0){
            if(editSelectedRobotWidget->isVisible()){
                setGraphicItemsState(GraphicItemState::NO_STATE);
                if(editSelectedRobotWidget->getRobotInfoDialog()->isVisible())
                    emit cancelRobotModifications();
                hideAllWidgets();
                leftMenu->hide();
            }


            /// if a box to save/edit this robot is open
            if(msgBox.isVisible())
                msgBox.close();

            selectedRobot = NULL;
        }

        /// if the robot is scanning
        if(scanningRobot != NULL && scanningRobot->getRobot()->getIp().compare(ip) == 0){
            editSelectedRobotWidget->getScanBtn()->setChecked(false);
            editSelectedRobotWidget->getScanBtn()->setText("Scan a map");
            editSelectedRobotWidget->setEnableAll(true);

            hideAllWidgets();
            setEnableAll(true);

            scanningRobot = NULL;
        }

        /// we stop the robots threads
        rv->getRobot()->deleteLater();

        /// delete robotview
        scene->removeItem(rv);

        /// remove from the model
        robots->remove(rv);

        rv->deleteLater();

        /// update robotsLeftWidget
        robotsLeftWidget->updateRobots(robots);

        /// bottomLayout
        bottomLayout->removeRobot(id);

        /// we remove the obstacles for this robot
        mapPixmapItem->getObstaclesPainter()->removeRobotObstacles(ip);

        qDebug() << "Done removing robot" << hostname << "at ip" << ip;
        setMessageTop(TEXT_COLOR_DANGER, QString("Robot " + hostname + " at ip " + ip +" disconnected."));
    } else {
        qDebug() << "(robotIsDeadSlot) A problem occured, the RobotView or its Robot are NULL, I have been kill twice ?";
    }
}

void MainWindow::setMessageCreationPath(QString message){
    setMessageTop(TEXT_COLOR_DANGER, message);
    delay(2500);
    if(selectedRobot)
        setMessageTop(TEXT_COLOR_INFO, "Click white points of the map to add new points to the path of " +
                  selectedRobot->getRobot()->getName() + "\nAlternatively you can click the \"+\" button to add an existing point to your path"
                  "\nYou can re-order the points in the list by dragging them");
    else
        setMessageTop(TEXT_COLOR_INFO, "Click white points of the map to add new points to your path\n"
                                       "Alternatively you can click the \"+\" button to add an existing point to your path"
                                       "\nYou can re-order the points in the list by dragging them");
}

void MainWindow::updateEditedPathPoint(double x, double y){
    qDebug() << "MainWindow::updateEditedPathPoint called";

    if(editedPointView)
        editedPointView->setPos(x, y);
    else
        qDebug() << "MainWindow::updateEditedPathPoint Could not find the pointView to edit";


    emit updatePathPainter(false);

    if(map->getMapImage().pixelColor(x, y).red() >= 254){
        setMessageTop(TEXT_COLOR_INFO, "You can click either \"Save changes\" to modify your path permanently or \"Cancel\" to keep the original path. If you want you can keep editing your point");

            static_cast<PathPointCreationWidget*> (pathCreationWidget->getPathPointList()->
                                               itemWidget(pathCreationWidget->getPathPointList()->currentItem()))
                                               -> getSaveEditBtn()->setEnabled(true);
    } else {
        setMessageTop(TEXT_COLOR_DANGER, "You cannot save the current path because the point that you are editing is not in a known area of the map");

        static_cast<PathPointCreationWidget*> (pathCreationWidget->getPathPointList()->
                                               itemWidget(pathCreationWidget->getPathPointList()->currentItem()))
                                               -> getSaveEditBtn()->setEnabled(false);
    }
}

void MainWindow::moveEditedPathPointSlot(){
    emit updatePathPainter(false);
    qDebug() << "MainWindow::moveEditedPathPointSlot";
    if(map->getMapImage().pixelColor(editedPointView->getPoint()->getPosition().getX(), editedPointView->getPoint()->getPosition().getY()).red() >= 254){
        setMessageTop(TEXT_COLOR_INFO, "You can click either \"Save changes\" to modify your path permanently or \"Cancel\" to keep the original path. If you want you can keep editing your point");
        static_cast<PathPointCreationWidget*> (pathCreationWidget->getPathPointList()->itemWidget(pathCreationWidget->getPathPointList()->currentItem()))->getSaveEditBtn()->setEnabled(true);
    } else {
        setMessageTop(TEXT_COLOR_DANGER, "You cannot save the current path because the point that you are editing is not in a known area of the map");
        static_cast<PathPointCreationWidget*> (pathCreationWidget->getPathPointList()->itemWidget(pathCreationWidget->getPathPointList()->currentItem()))->getSaveEditBtn()->setEnabled(false);
    }
}

void MainWindow::sendNewMapToRobots(QString ipAddress){
    qDebug() << "sendNewMapToRobots Map id and date :" << map->getMapId() << map->getDateTime().toString("yyyy-MM-dd-hh-mm-ss");

    QVector<QPointer<RobotView>> robotsVector = robots->getRobotsVector();

    /// We send the map to each robot
    for(int i = 0; i < robotsVector.size(); i++){
        QPointer<Robot> robot = robotsVector.at(i)->getRobot();

        /// No need to send the map to the robot that scanned it
        if(robot->getIp().compare(ipAddress) != 0){
            qDebug() << "Sending the map to" << robot->getName() << "at ip" << robot->getIp();
            robot->sendNewMap(map);
        } else {
            qDebug() << "The robot" << robot->getName() << "at ip" << robot->getIp() << "already has the current map";
        }
    }
    qDebug() << "Sent the map to the robots";
}

void MainWindow::updateAllPaths(const Point& old_point, const Point& new_point){
    qDebug() << "MainWindow::updateAllPaths called";
    for(int i = 0; i < robots->getRobotsVector().size(); i++){
        QPointer<Robot> robot = robots->getRobotsVector().at(i)->getRobot();
        /// to update the description of the path of each robot
        editSelectedRobotWidget->setSelectedRobot(robots->getRobotViewByName(robot->getName()));
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
        editSelectedRobotWidget->setAssignedPath(editSelectedRobotWidget->getAssignedPath());
        bottomLayout->updateRobot(robots->getRobotId(robot->getName()), robots->getRobotsVector().at(i));
    }

    if(editSelectedRobotWidget->getRobot()){
        /// to reset the tooltip of the edited point !
        int robotId = robots->getRobotId(editSelectedRobotWidget->getRobot()->getRobot()->getName());
        if(bottomLayout->getViewPathRobotBtnGroup()->button(robotId)->isChecked()){
            qDebug() << " i am displayed " << robotId;
            viewPathSelectedRobot(robotId, false);
            viewPathSelectedRobot(robotId, true);
        }
    }

    /// updates the paths of the model
    updateModelPaths(old_point, new_point);

    if(pathPainter->getVisiblePath().compare("")){
        /// to set the tooltip of the modified point
        for(int i = 0; i < pathPainter->getCurrentPath().size(); i++){
            Point currPoint = pathPainter->getCurrentPath().at(i)->getPoint();
            if(currPoint.comparePos(old_point.getPosition())){
                if(currPoint.comparePos(new_point.getPosition()))
                    currPoint.setName(new_point.getName());
            }
        }
        pathPainter->setCurrentPath(pathPainter->getCurrentPath(), pathPainter->getVisiblePath());
    }
}

void MainWindow::resetPathPointViewsSlot(){
    emit updatePathPainter(false);
}

void MainWindow::replacePoint(int id, QString name){
    points->getGroups()->value(PATH_GROUP_NAME)->remove(id);
    points->insertPoint(PATH_GROUP_NAME, id, points->findPointView(name));
}

void MainWindow::setNewHome(QString homeName){

    /// retrieves the pointview which name has been clicked in the menu
    QSharedPointer<PointView> home = points->findPointView(homeName);

    if(home->getPoint()->setHome(Point::PointType::HOME)){

        if(sendHomeToRobot(selectedRobot, home)){
            editSelectedRobotWidget->getGoHomeBtn()->show();

            /// Remove the previous home
            if(editSelectedRobotWidget->getHome()){
                editSelectedRobotWidget->getHome()->getPoint()->setHome(Point::PERM);
                editSelectedRobotWidget->getHome()->setPixmap(PointView::PixmapType::NORMAL);
            }
            savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

            /// associates the robot to the point
            home->getPoint()->setRobotName(selectedRobot->getRobot()->getName());

            /// saves the position of the new home in the corresponding file
            QFileInfo homeFileInfo(QDir::currentPath(), "../gobot-software/robots_homes/" + selectedRobot->getRobot()->getName());
            std::ofstream homeFile(homeFileInfo.absoluteFilePath().toStdString(), std::ios::out | std::ios::trunc);
            if(homeFile){
                homeFile << home->getPoint()->getPosition().getX() << " " << home->getPoint()->getPosition().getY();
                homeFile.close();
            }
            else
                qDebug() << "could not save home";

            editSelectedRobotWidget->setHome(home);
            selectedRobot->getRobot()->setHome(home);
            editSelectedRobotWidget->updateHomeMenu();

            /// so that if the new home if part of the path it displays the path correctly (not a house on top of a normal point)
            /// this call makes the home

            pathPainter->setCurrentPath(selectedRobot->getRobot()->getPath(), "");

            /// setCurrentPath is displaying the path so if it was not displayed we hide it
            if(!bottomLayout->getViewPathRobotBtnGroup()->button(robots->getRobotId(selectedRobot->getRobot()->getName()))->isChecked())
                emit resetPath();

            showSelectedRobotHomeOnly();

            home->setPixmap(PointView::PixmapType::SELECTED);
            home->show();

            setMessageTop(TEXT_COLOR_SUCCESS, selectedRobot->getRobot()->getName() + " successfully saved its home point");
        } else
            setMessageTop(TEXT_COLOR_DANGER, selectedRobot->getRobot()->getName() + " failed to save its home point, please try again");
    } else
        setMessageTop(TEXT_COLOR_DANGER, "Sorry, this point is already a home\nPlease select another");

}

bool MainWindow::sendHomeToRobot(QPointer<RobotView> robot, QSharedPointer<PointView> home){
    return commandController->sendCommand(robot->getRobot(), QString("n \"") + QString::number(home->getPoint()->getPosition().getX()) + "\" \""
                                                      + QString::number(home->getPoint()->getPosition().getY()) + "\"");
}

void MainWindow::goHome(){
    qDebug() << "MainWindow::goHome called (soon soon working)";
    if(commandController->sendCommand(selectedRobot->getRobot(), QString("o"))){
        //TODO change btn to pause the home (like for the path button)
        qDebug() << "MainWindow::goHome The robot" << selectedRobot->getRobot()->getName() << "is going home";
    } else {
        qDebug() << "MainWindow::goHome Failed to send the robot" << selectedRobot->getRobot()->getName() << "home, please try again";
    }
}

void MainWindow::goHome(int nbRobot){
    QPointer<Robot> currRobot = robots->getRobotsVector().at(nbRobot)->getRobot();
    if(!currRobot->isPlayingPath()){
        qDebug() <<"MainWindow::GoHome (bottomlayout) called";
        if(commandController->sendCommand(currRobot, QString("o"))){
            qDebug() << "MainWindow::goHome" << currRobot->getName() << "is going home";
        } else
            qDebug() << "MainWindow::goHome failed to send" << currRobot->getName() << "home";
    } else {
        int answer = openConfirmMessage("The robot " + currRobot->getName() + " is currently playing its path. Do you want to stop it and send it home anyway ?");
        switch(answer){
        case QMessageBox::Cancel:
            break;
        case QMessageBox::Yes:
            if(commandController->sendCommand(currRobot, QString("o"))){
                qDebug() << "MainWindow::goHome" << currRobot->getName() << "is going home";
            } else
                qDebug() << "MainWindow::goHome failed to send" << currRobot->getName() << "home";
            break;
        default:
            break;
        }
    }
}

/**********************************************************************************************************************************/

//                                          MAPS

/**********************************************************************************************************************************/

void MainWindow::updateMetadata(const int width, const int height, const float resolution,
                                const float originX, const float originY){

    bool modif = false;

    if(width != map->getWidth()){
        map->setWidth(width);
        modif = true;
    }

    if(height != map->getHeight()){
        map->setHeight(height);
        modif = true;
    }

    if(resolution != map->getResolution()){
        map->setResolution(resolution);
        modif = true;
    }

    if(originX != map->getOrigin().getX() || originY != map->getOrigin().getY()){
        map->setOrigin(Position(originX, originY));
        modif = true;
    }

    if(modif){
        qDebug() << "new path de la mort" << QDir::currentPath() + QDir::separator() + "mapConfigs" + QDir::separator() + QString::fromStdString(mapFile);
        saveMapConfig((QDir::currentPath() + QDir::separator() + "mapConfigs" + QDir::separator() + QString::fromStdString(mapFile)).toStdString());
        saveMapState();
    }
}

void MainWindow::updateMap(const QByteArray mapArray, bool fromPgm){
    /// TODO check if scanning or not and act accordingly
    map->setMapFromArray(mapArray, fromPgm);
    QPixmap pixmap = QPixmap::fromImage(map->getMapImage());
    mapPixmapItem->setPixmap(pixmap);
    /// WARNING might make the app send the map to every connected robot everytime we receive one while scanning
    /*map->setDateTime(QDateTime::currentDateTime());
    saveMapState();*/
    ///***************///
    scene->update();
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

        mapState.first = mapPixmapItem->pos();
        mapState.second = graphicsView->getZoomCoeff();

        saveMapState();

        QFileInfo mapFileInfo(static_cast<QDir> (fileName), "");
        QFileInfo fileInfo(QDir::currentPath(), "../gobot-software/mapConfigs/" + mapFileInfo.fileName() + ".config");
        qDebug() << fileInfo.absoluteFilePath();

        mapFile = fileName.toStdString() + ".pgm";

        assert(saveMapConfig(fileInfo.absoluteFilePath().toStdString()));

        qDebug() << "MainWindow::saveMap" << mapState.first.x() << mapState.first.y() << mapState.second << map->getWidth() << map->getHeight()
                 << map->getOrigin().getX() << map->getOrigin().getY();

        const QString pointsFile = fileName + "_points.xml";
        savePoints(pointsFile);

        /// saves the map
        map->saveToFile(fileName + ".pgm");

        const QString pathsFile = fileName + "_paths.dat";

        /// saves the current configuration for the paths (this configuration will be associated to the map
        /// when you load the map in the future
        serializePaths(pathsFile);
    }
}

void MainWindow::loadMapBtnEvent(){
    qDebug() << "loadMapBtnEvent called";
    QMessageBox box;
    box.setText("Warning, loading a new map will erase all previously created points, paths and selected home of robots. Do you wish to save your current configuration first ?");
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
        assert(loadMapConfig(fileInfo.absoluteFilePath().toStdString()));

        /// clears the map of all paths and points
        clearNewMap();

        qDebug() << "about to load map from" << QString::fromStdString(mapFile);
        map->setMapFromFile(QString::fromStdString(mapFile));
        setWindowTitle(QString::fromStdString(mapFile));

        map->setDateTime(QDateTime::currentDateTime());
        saveMapState();

        QPixmap pixmap = QPixmap::fromImage(map->getMapImage());
        mapPixmapItem->setPixmap(pixmap);
        scene->update();

        /// centers the map
        centerMap();

        /// imports paths associated to the map and save them in the current file
        deserializePaths(fileNameWithoutExtension + "_paths.dat");

        /// imports points associated to the map and save them in the current file
        XMLParser parser(fileNameWithoutExtension + "_points.xml");
        parser.readPoints(points);

        /// savesthe new configuration to the current configuration file
        savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

        /// updates the group box so that new points can be added
        createPointWidget->updateGroupBox();

        serializePaths(QDir::currentPath() + QDir::separator() + "paths.dat");

        /// updates the groups of paths menu using the paths that have just been imported
        leftMenu->getGroupsPathsWidget()->updateGroupsPaths();
    }
}

void MainWindow::mapBtnEvent(){
    qDebug() << "MainWindow::mapBtnEvent called";
    leftMenuWidget->hide();
    mapLeftWidget->show();
    switchFocus("Menu", mapLeftWidget, MainWindow::WidgetType::MAP);
}

void MainWindow::messageMapSaved(bool status){
    if(status)
        setTemporaryMessageTop(TEXT_COLOR_SUCCESS, "You have successfully saved the map", 2500);
    else
        setTemporaryMessageTop(TEXT_COLOR_DANGER, "Attempt to save the map failed", 2500);
}

void MainWindow::editMapSlot(){
    editMapWidget = QPointer<EditMapWidget>(new EditMapWidget(map->getMapImage(), map->getWidth(), map->getHeight(), map->getResolution(), map->getOrigin()));
    connect(editMapWidget, SIGNAL(saveEditMap()), this, SLOT(saveEditMapSlot()));
}

void MainWindow::saveEditMapSlot(){
    qDebug() << "MainWindow::saveEditMapSlot called";
    if(editMapWidget){
        QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
                "", tr("Images (*.pgm)"));

        /// If the user perss cancel on the box, the filename fill be empty
        if(!fileName.isEmpty()){
            qDebug() << "MainWindow::saveEditMapSlot called with filename" << fileName;

            mapFile = fileName.toStdString();

            /// Set the new map
            map->setResolution(editMapWidget->getResolution());
            map->setWidth(editMapWidget->getWidth());
            map->setHeight(editMapWidget->getHeight());
            map->setOrigin(editMapWidget->getOrigin());
            map->setMapImage(editMapWidget->getMapImage());
            map->setMapId(QUuid::createUuid());
            map->setDateTime(QDateTime::currentDateTime());

            QPixmap pixmap = QPixmap::fromImage(editMapWidget->getMapImage());
            mapPixmapItem->setPixmap(pixmap);
            scene->update();

            saveMap(fileName);
            editMapWidget->close();

            sendNewMapToRobots();
        }
    } else {
        qDebug() << "MainWindow::saveEditMapSlot called while editMapWidget is not set, duh ?";
    }
}

/**********************************************************************************************************************************/

//                                          MENUS

/**********************************************************************************************************************************/


void MainWindow::initializeLeftMenu(){
    lastWidgets =  QList<QPair<QPair<QWidget*,QString>, MainWindow::WidgetType>>();
    leftMenuWidget = leftMenu->getLeftMenuWidget();
    pointsLeftWidget = leftMenu->getPointsLeftWidget();
    robotsLeftWidget = leftMenu->getRobotsLeftWidget();
    mapLeftWidget = leftMenu->getMapLeftWidget();
    editSelectedRobotWidget = leftMenu->getEditSelectedRobotWidget();
    selectedPointWidget = leftMenu->getSelectedPointWidget();
    createPointWidget = leftMenu->getEditSelectedPointWidget();
    pathCreationWidget = leftMenu->getpathCreationWidget();
}

void MainWindow::initializeBottomPanel(){
    bottomLayout = new BottomLayout(this, robots);
    rightLayout->addWidget(bottomLayout);
}

void MainWindow::setMessageTop(const QString msgType, const QString msg){
    topLayout->setLabel(msgType, msg);
}

void MainWindow::closeSlot(){
    resetFocus();
    leftMenu->getDisplaySelectedPoint()->hide();
    leftMenu->hide();
    setEnableAll(true);
    if(leftMenu->getDisplaySelectedPoint()->getPointView())
        leftMenu->getDisplaySelectedPoint()->getPointView()->setPixmap(PointView::PixmapType::NORMAL);
}

/**********************************************************************************************************************************/

//                                          POINTS

/**********************************************************************************************************************************/

/**
 * @brief MainWindow::initializePoints
 * initialize the points on the map and in the model
 */
void MainWindow::initializePoints(){
    /// retrieves the points from the xml file and stores them in the model
    XMLParser pParser(QDir::currentPath() + QDir::separator() + "points.xml");
    qDebug() << "initializing points from" << QDir::currentPath() + QDir::separator() + "points.xml";
    pParser.readPoints(points);
    points->addTmpPoint();
    mapPixmapItem->setPoints(points);
}

void MainWindow::savePoints(const QString fileName){
    XMLParser parser(fileName);
    parser.save(*points);
}

/**
 * @brief MainWindow::setSelectedPoint
 * @param pointView
 * @param isTemporary
 * set the selected point, could be a temporary point or a point that already exists and that might be edited
 */
void MainWindow::setSelectedPoint(){
    qDebug() << "MainWindow::setSelectedPoint called";

    resetFocus();

    createPointWidget->getGroupBox()->hide();
    createPointWidget->getGroupLabel()->hide();

    QSharedPointer<PointView> displaySelectedPointView = points->getTmpPointView();

    /// sets the pixmaps of the other points
    points->setPixmapAll(PointView::NORMAL);

    displaySelectedPointView->setPixmap(PointView::MID);

    int id = bottomLayout->getViewPathRobotBtnGroup()->checkedId();
    if(id > 0)
        pathPainter->setCurrentPath(robots->getRobotsVector().at(id)->getRobot()->getPath(), "");

    leftMenu->show();

    hideAllWidgets();
    createPointWidget->setSelectedPoint(displaySelectedPointView);
    createPointWidget->show();
    float x = displaySelectedPointView->getPoint()->getPosition().getX();
    float y = displaySelectedPointView->getPoint()->getPosition().getY();

    if(map->getMapImage().pixelColor(x, y).red() >= 254){
        setMessageTop(TEXT_COLOR_INFO, "To save this point permanently click the \"+\" button");
        createPointWidget->getActionButtons()->getPlusButton()->setEnabled(true);
        createPointWidget->getActionButtons()->getPlusButton()->setToolTip("Click this button if you want to save this point permanently");
    } else {
        setMessageTop(TEXT_COLOR_WARNING, "You cannot save this point because your robot(s) would not be able to go there");
        createPointWidget->getActionButtons()->getPlusButton()->setEnabled(false);
        createPointWidget->getActionButtons()->getPlusButton()->setToolTip("You cannot save this point because your robot(s) cannot go there");
    }

    leftMenu->getDisplaySelectedPoint()->hide();
    switchFocus(displaySelectedPointView->getPoint()->getName(), createPointWidget, MainWindow::WidgetType::POINT);

    /// to deselect a potentially selected robot
    bottomLayout->uncheckRobots();
    robots->deselect();
    bottomLayout->setLastCheckedId(-1);
}

/**
 * @brief MainWindow::pointBtnEvent
 * called when the back button is clicked
 */
void MainWindow::pointBtnEvent(void){
    /// resets the list of groups menu
    switchFocus("Groups", pointsLeftWidget, MainWindow::WidgetType::GROUPS);
    qDebug() << "pointBtnEvent called ";
    /// we uncheck all buttons from all menus
    leftMenu->getDisplaySelectedGroup()->uncheck();
    hideAllWidgets();
    pointsLeftWidget->show();
    setMessageTop(TEXT_COLOR_INFO, "Click the map to add a permanent point");
}

void MainWindow::plusGroupBtnEvent(){
    setMessageTop(TEXT_COLOR_INFO, "The name of your group cannot be empty");
    qDebug() << "plusGroupBtnEvent called";

    pointsLeftWidget->getGroupNameEdit()->setFocus();
    pointsLeftWidget->setCreatingGroup(true);
    pointsLeftWidget->getGroupButtonGroup()->uncheck();
    /// resets the name edit field
    pointsLeftWidget->getGroupNameEdit()->setText("");
    /// stops a user from creating a new group with no name
    pointsLeftWidget->getSaveButton()->setEnabled(false);
    /// uncheck and disable the buttons
    pointsLeftWidget->getActionButtons()->checkAll(false);

    pointsLeftWidget->getActionButtons()->enableAll(false);

    pointsLeftWidget->getActionButtons()->getPlusButton()->setToolTip("Enter a name for your group and click \"save\" or click \"cancel\" to cancel");

    /// to prevent the user from clicking on the buttons
    pointsLeftWidget->getGroupButtonGroup()->setEnabled(false);

    /// here we allow a user to create a new group

    pointsLeftWidget->getGroupNameLabel()->show();

    pointsLeftWidget->getGroupNameEdit()->show();

    pointsLeftWidget->getCancelButton()->show();
    pointsLeftWidget->getSaveButton()->show();

    pointsLeftWidget->getGroupNameEdit()->setFocus();
}

/**
 * @brief MainWindow::minusGroupBtnEvent
 * called in the first points menu to either remove a group or a point which belongs to the default group
 */
void MainWindow::minusGroupBtnEvent(){
    qDebug() << "minusGroupBtnEvent called";

    /// uncheck the other buttons
    pointsLeftWidget->getActionButtons()->getPlusButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getEditButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getGoButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getMapButton()->setChecked(false);

    /// we hide those in case the previous button clicked was the plus button
    pointsLeftWidget->getGroupNameEdit()->hide();
    pointsLeftWidget->getGroupNameLabel()->hide();

    QString checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton()->text();

    /// we have to delete a group
    if(points->isAGroup(checkedId))
        askForDeleteGroupConfirmation(checkedId);
    /// we have to delete a point
    else if(points->isAPoint(checkedId))
        askForDeleteDefaultGroupPointConfirmation(checkedId);

    leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->setGroup(leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getGroupName());
    leftMenu->getPointsLeftWidget()->updateGroupButtonGroup();
}

/**
 * @brief MainWindow::editPointButtonEvent
 * @param checked
 * called to edit an existing point
 */
void MainWindow::editPointButtonEvent(){
    setMessageTop(TEXT_COLOR_INFO, "Click the map or drag the point to change its position");
    qDebug() << "MainWindow::editPointButtonEvent called";
    leftMenu->getDisplaySelectedPoint()->getNameEdit()->show();
    leftMenu->getDisplaySelectedPoint()->getNameLabel()->hide();

    /// update buttons enable attribute and tool tips
    leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMapButton()->setChecked(true);
    leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMinusButton()->setEnabled(false);
    leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMinusButton()->setToolTip("");
    leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMapButton()->setEnabled(false);
    leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMapButton()->setToolTip("");

    /// hide the temporary point on the map
    QSharedPointer<PointView> displaySelectedPointView = points->findPointView(leftMenu->getDisplaySelectedPoint()->getPointName());
    qDebug() << "MainWindow::editPointButtonEvent selected point to edit " << leftMenu->getDisplaySelectedPoint()->getPointName();
    displaySelectedPointView->setOriginalPosition(displaySelectedPointView->getPoint()->getPosition());

    if(displaySelectedPointView && !(*(displaySelectedPointView->getPoint()) == *(points->getTmpPointView()->getPoint())))
        points->displayTmpPoint(false);

    /// change the color of the pointview that's selected on the map
    displaySelectedPointView->setPixmap(PointView::PixmapType::SELECTED);
    displaySelectedPointView->show();

    pointsLeftWidget->getActionButtons()->getPlusButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getMinusButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getGoButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getMapButton()->setChecked(false);

    /// this way we force the user to either click save or cancel
    leftMenu->getDisplaySelectedPoint()->getActionButtons()->getEditButton()->setEnabled(false);
    leftMenu->getDisplaySelectedPoint()->getActionButtons()->getEditButton()->setToolTip("You can choose to save or discard your modifications by clicking the save (Enter) and cancel button respectively");

    /// we show the save button and the cancel button
    leftMenu->getDisplaySelectedPoint()->getCancelButton()->show();
    leftMenu->getDisplaySelectedPoint()->getSaveButton()->show();

    /// sets the state of the map and the other widgets to prevent other concurrent actions
    setGraphicItemsState(GraphicItemState::NO_EVENT);
    mapPixmapItem->setState(GraphicItemState::EDITING_PERM);

    /// sets the state of the point of the map to make it draggable
    displaySelectedPointView->setState(GraphicItemState::EDITING_PERM);
    displaySelectedPointView->setFlag(QGraphicsItem::ItemIsMovable, true);

    leftMenu->getDisplaySelectedPoint()->getNameEdit()->setText("");
    leftMenu->getDisplaySelectedPoint()->getNameEdit()->setPlaceholderText(displaySelectedPointView->getPoint()->getName());
    leftMenu->getDisplaySelectedPoint()->getNameEdit()->setFocus();
}

/**
 * @brief MainWindow::editGroupBtnEvent
 * @param checked
 * called when the user wants to edit a point from the first points menu
 */
void MainWindow::editGroupBtnEvent(){

    if(pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton()){
        qDebug() << "editGroupBtnEvent called" << pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton()->text();

        int btnIndex = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedId();

        pointsLeftWidget->setLastCheckedId(static_cast<CustomPushButton*>(pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton())->text());
        assert(pointsLeftWidget->getLastCheckedId().compare(""));
        pointsLeftWidget->setCreatingGroup(false);

        /// uncheck the other buttons
        pointsLeftWidget->getActionButtons()->getPlusButton()->setChecked(false);
        pointsLeftWidget->getActionButtons()->getMinusButton()->setChecked(false);
        pointsLeftWidget->getActionButtons()->getGoButton()->setChecked(false);
        pointsLeftWidget->getActionButtons()->getMapButton()->setChecked(false);

        /// we hide those in case the previous button clicked was the plus button
        pointsLeftWidget->getGroupNameEdit()->hide();
        pointsLeftWidget->getGroupNameLabel()->hide();
        QAbstractButton* btn = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton();
        QString checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton()->text();

        /// it's an isolated point
        if(checkedId.compare("") != 0 && points->isAPoint(checkedId)){

            /// retrieves the pointview associated to the point on the map and displays it if it was not already the case
            QSharedPointer<PointView> pointView = points->findPointView(checkedId);

            /// must display the tick icon in the pointsLeftWidget
            pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton()->setIcon(QIcon(":/icons/eye_point.png"));
            if(pointView){
                pointView->show();
                QString robotName = "";
                if(pointView->getPoint()->isHome()){
                    QPointer<RobotView> rv = robots->findRobotUsingHome(pointView->getPoint()->getName());
                    if(rv != NULL)
                        robotName = rv->getRobot()->getName();
                    else
                        qDebug() << "editGroupBtnEvent : something unexpected happened";
                }
                leftMenu->getDisplaySelectedPoint()->setPointView(pointView, robotName);
            } else {
                qDebug() << "There is no point view associated with those indexes";
            }

            /// displays the information relative the the point
            leftMenu->getDisplaySelectedPoint()->displayPointInfo();
            editPointButtonEvent();
            pointsLeftWidget->hide();


            /// disables the back button to prevent problems, a user has to discard or save his modifications before he can start navigatin the menu again, also prevents false manipulations
            leftMenu->getDisplaySelectedPoint()->show();
            switchFocus("point", leftMenu->getDisplaySelectedPoint(), MainWindow::WidgetType::POINT);
        } else if(checkedId.compare("") != 0 && points->isAGroup(checkedId)){
            qDebug() << "gotta update a group";

            pointsLeftWidget->getActionButtons()->getEditButton()->setToolTip("Type a new name for your group and press ENTER");
            /// disables the plus button
            pointsLeftWidget->getActionButtons()->getPlusButton()->setEnabled(false);
            /// disables the other buttons
            pointsLeftWidget->disableButtons();
            pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->setText("");

            pointsLeftWidget->getGroupButtonGroup()->uncheck();
            pointsLeftWidget->getGroupButtonGroup()->setEnabled(false);

            pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->setText("");
            pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->setPlaceholderText(checkedId);
            pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->setFocus();
            pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->setFocusPolicy(Qt::FocusPolicy::StrongFocus);
            pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->show();
            pointsLeftWidget->getGroupButtonGroup()->getLayout()->removeWidget(pointsLeftWidget->getGroupButtonGroup()->getModifyEdit());
            pointsLeftWidget->getGroupButtonGroup()->getLayout()->insertWidget(btnIndex, pointsLeftWidget->getGroupButtonGroup()->getModifyEdit());

            pointsLeftWidget->getGroupButtonGroup()->setEditedGroupName(checkedId);
            btn->hide();
        }
    }
}

void MainWindow::switchFocus(const QString name, QWidget* widget, const MainWindow::WidgetType type)
{
    lastWidgets.append(QPair<QPair<QWidget*, QString>, MainWindow::WidgetType>(QPair<QWidget*, QString>(widget,name), type));

    if(lastWidgets.size() > 1)
        leftMenu->showBackButton(lastWidgets.at(lastWidgets.size()-2).first.second);
    else
        leftMenu->hideBackButton();

    qDebug() << "__________________";

    for(int i = 0; i < lastWidgets.size(); i++)
        qDebug() << lastWidgets.at(i).first.second;

    qDebug() << "_________________";
}

void MainWindow::resetFocus()
{
    lastWidgets = QList<QPair<QPair<QWidget*,QString>, MainWindow::WidgetType>>();
    updateView();
}

void MainWindow::updateView()
{
    if (leftMenu != NULL)
    {
        if(lastWidgets.size() <= 1){
            leftMenu->hideBackButton();
        }
        else {
            leftMenu->showBackButton(lastWidgets.last().first.second);
        }
    }
}

void MainWindow::openLeftMenu(){
    qDebug() << "openLeftMenu called";
    setMessageTop(TEXT_COLOR_NORMAL, "");

    /// resets the color of the selected point on the map and hides the temporary point`
    if(leftMenu->getDisplaySelectedPoint()->getPointView()){
        QSharedPointer<PointView> displaySelectedPointView = points->findPointView(leftMenu->getDisplaySelectedPoint()->getPointName());
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
        leftMenu->getDisplaySelectedPoint()->hide();
        if(leftMenuWidget->isHidden()){
            hideAllWidgets();
            leftMenuWidget->show();
            leftMenu->show();
            switchFocus("Menu",leftMenuWidget, MainWindow::WidgetType::MENU);
        } else {
                closeSlot();
        }
    }
}

void MainWindow::minusSelecPointBtnEvent(){
    qDebug() << "minusSelecPointBtnEvent called";
}

void MainWindow::editSelecPointBtnEvent(){
    qDebug() << "editSelecPointBtnEvent called";
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

    /// resets the status of the plus button
    createPointWidget->getActionButtons()->getPlusButton()->setEnabled(true);

    /// hides widgets relative to the choice of a group
    createPointWidget->hideGroupLayout(true);

    points->addPoint(groupName, name, x, y, true, Point::PointType::TEMP);

    /// saves it to the file
    savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

    /// updates the menu
    pointsLeftWidget->updateGroupButtonGroup();

    /// hides the temporary point so that they don't superimpose which is confusing when hiding / showing the newly created point
    points->getTmpPointView()->hide();

    /// hide the creation widget
    hideAllWidgets();
    leftMenu->hide();

    setMessageTop(TEXT_COLOR_SUCCESS, "You have successfully added the new point \"" + name + "\" to the group : \"" + groupName + "\"");
}

/**
 * @brief MainWindow::askForDeleteDefaultGroupPointConfirmation
 * @param pointName
 * Called when a user wants to remove a point that belongs to the default group
 * the pointName given is the name of the point within its group
 */
void MainWindow::askForDeleteDefaultGroupPointConfirmation(QString pointName){
    qDebug() << "askForDeleteDefaultGroupPointConfirmation called" << pointName;
    int ret = openConfirmMessage("Do you really want to remove this point ?");
    switch(ret){
        case QMessageBox::Cancel :
            pointsLeftWidget->getActionButtons()->getMinusButton()->setChecked(false);
            pointsLeftWidget->setLastCheckedId("");
            pointsLeftWidget->disableButtons();
            points->setPixmapAll(PointView::PixmapType::NORMAL);
        break;
        case QMessageBox::Ok : {
            /// we first check that our point is not the home of a robot
            QSharedPointer<Point> point = points->findPoint(pointName);
            if(!point->isHome()){
                qDebug() << "Go ahead and remove me I am not a home point anyway";

                /// updates the model
                points->removePoint(pointName);

                /// save changes in the file
                savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

                /// updates the menu
                pointsLeftWidget->getGroupButtonGroup()->updateButtons();

                /// need to remove the point from the map
                pointsLeftWidget->setLastCheckedId("");
            } else {
                /// this is in fact the home point of a robot, we prompt a customized message to the end user
                QPointer<RobotView> robot = robots->findRobotUsingHome(pointName);
                if(robot != NULL){
                    openInterdictionOfPointRemovalMessage(pointName, robot->getRobot()->getName());
                    qDebug() << "Sorry this point is the home of a robot and therefore cannot be removed";
                } else {
                    qDebug() << "setSelectedRobotFromPoint : something unexpected happened";
                }
                points->setPixmapAll(PointView::PixmapType::NORMAL);
            }
        }
        pointsLeftWidget->disableButtons();
        break;
        default:
            Q_UNREACHABLE();
        /// should never be here
            qDebug() << "MainWindow::askForDeleteDefaultGroupPointConfirmation should not be here";
        break;
    }
}

/**
 * @brief MainWindow::askForDeletePointConfirmation
 * @param pointName
 * Called when a user wants to remove a point that belongs to any group but the default one
 * the pointName given is the name of the point within its group
 */
void MainWindow::askForDeletePointConfirmation(QString pointName){
    qDebug() << "askfordeletepointconfirmation event called" << pointName;
    int ret = openConfirmMessage("Do you really want to remove this point ?");
    switch(ret){
        case QMessageBox::Cancel :
            qDebug() << "clicked no";
            leftMenu->getDisplaySelectedGroup()->disableButtons();
        break;
        case QMessageBox::Ok :
            {
                leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->setCheckable(true);
                /// we first check that our point is not the home of a robot
                qDebug() << "pointName:" << pointName;

                QSharedPointer<PointView> point = points->findPointView(pointName);
                QString group = points->getGroupNameFromPointName(pointName);
                if(point && !point->getPoint()->isHome()){
                    qDebug() << "Go ahead and remove me I am not a home point anyway";
                    /// need to remove the point from the map
                    point->hide();

                    /// updates the model
                    points->removePoint(pointName);

                    /// updates the group menu
                    leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->setGroup(pointsLeftWidget->getLastCheckedId());

                    /// save the changes to the file
                    savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

                    /// makes the buttons checkable again
                    leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->setCheckable(true);

                    /// prompts the user to ask him if he wants to delete the group in case it would be empty
                    if(points->getGroups()->value(group)->size() <= 0){
                        int res = openEmptyGroupMessage(group);
                        if(res == QMessageBox::Yes){
                            /// updates model
                            points->removeGroup(pointsLeftWidget->getLastCheckedId());

                            /// updates file
                            savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

                            /// updates menu
                            pointsLeftWidget->getGroupButtonGroup()->updateButtons();

                            /// hides group menu and shows list of groups menu
                            leftMenu->getDisplaySelectedGroup()->hide();
                            pointsLeftWidget->show();
                            createPointWidget->updateGroupBox();
                            backEvent();
                        }
                    }

                } else {
                    /// this is in fact the home point of a robot, we prompt a customized message to the end user
                    QPointer<RobotView> robot = robots->findRobotUsingHome(pointName);
                    if(robot != NULL){
                        qDebug() << robot->getRobot()->getName();
                        openInterdictionOfPointRemovalMessage(pointName, robot->getRobot()->getName());
                        qDebug() << "Sorry this point is the home of a robot and therefore cannot be removed";
                    } else {
                        qDebug() << "askForDeletePointConfirmation : something unexpected happened";
                    }
                }
                points->setPixmapAll(PointView::PixmapType::NORMAL);
                leftMenu->getDisplaySelectedGroup()->disableButtons();
                if(point && !point->getPoint()->isHome())
                    setTemporaryMessageTop(TEXT_COLOR_SUCCESS, "You have successfully deleted the point \"" + pointName + "\"", 2500);
            }

        break;
        default:
            Q_UNREACHABLE();
        /// should never be here
            qDebug() << "MainWindow::askForDeletePointConfirmation should not be here";
        break;
    }
}

/**
 * @brief MainWindow::askForDeleteGroupConfirmation
 * @param groupName
 * Called when a user wants to remove a whole group of points
 */
void MainWindow::askForDeleteGroupConfirmation(QString groupName){
    qDebug() << "askForDeleteGroupConfirmation called";
    int ret = openConfirmMessage("Do you really want to remove this group ? All the points in this group will also be removed.");
    switch(ret){
        case QMessageBox::Cancel :
            qDebug() << "clicked no ici la tt suite";
            pointsLeftWidget->getActionButtons()->getMinusButton()->setChecked(false);
            pointsLeftWidget->setLastCheckedId("");
            pointsLeftWidget->disableButtons();
            points->setPixmapAll(PointView::PixmapType::NORMAL);
        break;
        case QMessageBox::Ok : {
            /// we have to check that none of the points is the home of a robot
            QVector<QString> homePointNames = points->getHomeNameFromGroup(groupName);
            if(homePointNames.size() <= 0){

                /// removes all the points of the group on the map
                for(int i = 0; i < points->getGroups()->value(groupName)->size(); i++){
                    points->getGroups()->value(groupName)->at(i)->hide();
                }

                /// removes the group from the model
                points->removeGroup(groupName);

                /// updates the file
                savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

                /// updates the group box so that the user cannot create a point in this group anymore
                createPointWidget->updateGroupBox();

            } else {
                /// this group contains the home point of a robot and cannot be removed,
                /// we prompt the end user with a customized message to explain
                /// which robot has its home point in the group
                QString pointsNameStr = "";
                for(int i = 0; i < homePointNames.size(); i++){
                    if(i > 0)
                        pointsNameStr += ", ";
                    pointsNameStr += homePointNames.at(i);
                }
                QMessageBox msgBox;
                msgBox.setText("This group contains the point(s) : " + pointsNameStr
                               + " which are home to their respective robot."
                               + " If you want to remove it you first have to indicate a new home point for this robot.");
                msgBox.setIcon(QMessageBox::Critical);
                msgBox.setStandardButtons(QMessageBox::Ok);
                msgBox.setInformativeText("To modify the home point of a robot you can either click on the menu > Robots, choose a robot and Add Home or simply click a robot on the map and Add Home");
                msgBox.exec();
                qDebug() << "Sorry this point is the home of a robot and therefore cannot be removed";

            }

            /// updates the menu
            pointsLeftWidget->getGroupButtonGroup()->updateButtons();
            pointsLeftWidget->getActionButtons()->getMinusButton()->setChecked(false);
            pointsLeftWidget->setLastCheckedId("");
            pointsLeftWidget->disableButtons();
            points->setPixmapAll(PointView::PixmapType::NORMAL);

            if(homePointNames.size() <= 0)
                setTemporaryMessageTop(TEXT_COLOR_SUCCESS, "You have successfully deleted the group \"" + groupName + "\"", 2500);
        }
        break;
        default:
            Q_UNREACHABLE();
            /// should never be here
            Q_UNREACHABLE();
            qDebug() << " dafuk ?";
        break;
    }
}

void MainWindow::displayPointEvent(QString name, double x, double y){
    qDebug() << "MainWindow::displayPointEvent called" << name;
    QSharedPointer<PointView> pointView = points->findPointView(name);
    if(!pointView)
        pointView = points->findPathPointView(x, y);

    if(pointView){
        if(!(*(pointView->getPoint()) == *(points->getTmpPointView()->getPoint()))){
            /// If the point is not a path or is a path but from a permanent point, we display the menu with informations on the point
            if(!(pointView->getPoint()->isPath() && pointView->getPoint()->getName().contains(PATH_POINT_NAME))){

                points->displayTmpPoint(false);

                leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMapButton()->setChecked(true);

                QString robotName = "";
                if(pointView->getPoint()->isHome()){
                    QPointer<RobotView> rv = robots->findRobotUsingHome(pointView->getPoint()->getName());
                    if(rv != NULL)
                        robotName = rv->getRobot()->getName();
                    else
                        qDebug() << "MainWindow::displayPointEvent : something unexpected happened";
                }

                leftMenu->getDisplaySelectedPoint()->setPointView(pointView, robotName);

                /// so that the points don't stay blue if we click a new point
                points->setPixmapAll(PointView::PixmapType::NORMAL);

                pointView->setState(GraphicItemState::NO_STATE);

                leftMenu->getDisplaySelectedPoint()->displayPointInfo();

                hideAllWidgets();

                leftMenu->show();

                leftMenu->getDisplaySelectedPoint()->show();
                resetFocus();
                switchFocus(pointView->getPoint()->getName(), leftMenu->getDisplaySelectedPoint(), MainWindow::WidgetType::POINT);

                qDebug() << "MainWindow::displayPointEvent  : is this point a path ?" << (pointView->getPoint()->isPath()) << pointView->getPoint()->getType();
                if(pointView->getPoint()->isHome()){
                    QPointer<RobotView> rv = robots->findRobotUsingHome(pointView->getPoint()->getName());
                    if(rv != NULL)
                        robotName = rv->getRobot()->getName();
                    else
                        qDebug() << "MainWindow::displayPointEvent  : something unexpected happened";
                }

                if(pointView->getPoint()->isPath()){
                    /// if it's a path point the edition/suppression is forbidden from here
                    leftMenu->getDisplaySelectedPoint()->getActionButtons()->getEditButton()->setEnabled(false);
                    leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMinusButton()->setEnabled(false);
                }

            } else {
                /// The point is a path' point from a temporary point so we display the page of the robot in which this pathpoint is used
                QPointer<RobotView> robot = robots->findRobotUsingTmpPointInPath(pointView->getPoint());
                if(robot){
                    qDebug() << "MainWindow::displayPointEvent  At least, I found the robot" << robot->getRobot()->getName();
                    resetFocus();
                    setSelectedRobot(robot);
                }
            }
        } else {
            /// It's the tmpPoint
            setSelectedPoint();
        }
    } else {
        qDebug() << "MainWindow::displayPointEvent could not found the pointView" << name << x << y;
    }
}

void MainWindow::displayGroupMapEvent(void){
    qDebug() << "MainWindow::displayGroupMapEvent called";

    if(pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton()){
        /// uncheck the other buttons
        pointsLeftWidget->getActionButtons()->getPlusButton()->setChecked(false);
        pointsLeftWidget->getActionButtons()->getMinusButton()->setChecked(false);
        pointsLeftWidget->getActionButtons()->getEditButton()->setChecked(false);
        pointsLeftWidget->getActionButtons()->getGoButton()->setChecked(false);

        /// we hide those in case the previous button clicked was the plus button
        pointsLeftWidget->getGroupNameEdit()->hide();
        pointsLeftWidget->getGroupNameLabel()->hide();

        QString checkedName = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton()->text();

        int checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedId();
        /// we display groups
        if(points->isAGroup(checkedName)){
            if(points->getGroups()->value(checkedName)){
                /// the group was displayed, we now have to hide it (all its points)
                if(points->isDisplayed(checkedName)){

                    /// updates the tooltip of the map button
                    pointsLeftWidget->getActionButtons()->getMapButton()->setToolTip("Click here to display the selected group on the map");
                    pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/folder_space.png"));

                    for(int i = 0; i < points->getGroups()->value(checkedName)->size(); i++){
                        QSharedPointer<PointView> point = points->getGroups()->value(checkedName)->at(i);
                        point->hide();

                    }
                    /// update the file
                    savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

                } else if(points->getGroups()->value(checkedName)->size() == 0) {
                    pointsLeftWidget->getActionButtons()->getMapButton()->setChecked(false);
                    topLayout->setLabelDelay(TEXT_COLOR_WARNING, "This group is empty. There is no points to display", 4000);
                } else {
                    /// updates the tooltip of the map button
                    pointsLeftWidget->getActionButtons()->getMapButton()->setToolTip("Click here to hide the selected group on the map");
                    /// the group must now be displayed
                    pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/folder_eye.png"));

                    for(int i = 0; i < points->getGroups()->value(checkedName)->size(); i++){
                        QSharedPointer<PointView> point = points->getGroups()->value(checkedName)->at(i);
                        point->show();
                        point->setPixmap(PointView::PixmapType::SELECTED);
                        /// update the file
                        savePoints(QDir::currentPath() + QDir::separator() + "points.xml");
                    }
                }
            }
        }
        /// we display isolated points
        else if(points->isAPoint(checkedName)){
            QSharedPointer<PointView> point = points->findPointView(checkedName);

            /// if the point is displayed we hide it
            if(point && point->isVisible()){
                /// updates the tooltip of the map button
                pointsLeftWidget->getActionButtons()->getMapButton()->setToolTip("Click here to display the selected point on the map");
                point->hide();

                /// update the file
                savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

                /// we remove the tick icon
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/space_point.png"));
            } else {
                /// updates the tooltip of the map button
                pointsLeftWidget->getActionButtons()->getMapButton()->setToolTip("Click here to hide the selected point on the map");
                point->setPixmap(PointView::PixmapType::SELECTED);
                point->show();

                /// update the file
                savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

                /// we add the tick icon
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/eye_point.png"));
            }
        }
    }
}

void MainWindow::displayPointMapEvent(){
    qDebug() << "MainWindow::displayPointMapEvent called";
    QSharedPointer<PointView> pointView = points->findPointView(leftMenu->getDisplaySelectedPoint()->getPointName());
    QPair<QString, int> pointIndexes = points->findPointIndexes(pointView->getPoint()->getName());
    qDebug() << "Indexes are " << pointIndexes.first << pointIndexes.second;
    int checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonIdByName(pointIndexes.first);

    if(pointView && pointView->getPoint()){
        if(pointView->isVisible()){
            qDebug() << "MainWindow::displayPointMapEvent hiding" << pointView->getPoint()->getName();
            leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMapButton()->setToolTip("Click to display this point");
            pointView->hide();

            /// update the file
            savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

            /// we update the group menu
            leftMenu->updateGroupDisplayed(pointIndexes.first);

            /// it's a point that belongs to a group
            if(pointIndexes.first.compare(NO_GROUP_NAME) != 0){
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/folder_space.png"));
                qDebug() << pointIndexes.second;
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/space_point.png"));
            } else {
                /// it's an isolated point
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(pointIndexes.second)->setIcon(QIcon(":/icons/space_point.png"));
            }
        } else {
            qDebug() << "MainWindow::displayPointMapEvent showing" << pointView->getPoint()->getName();
            pointView->setPixmap(PointView::PixmapType::SELECTED);
            leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMapButton()->setToolTip("Click to hide this point");
            pointView->show();

            /// update the file
            savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

            /// we update the groups menu
            /// it's a point that belongs to a group
            if(pointIndexes.first.compare(NO_GROUP_NAME) != 0){
                qDebug() << pointIndexes.second;
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/eye_point.png"));
                /// we check whether or not the entire group is displayed and update the points left widget accordingly by adding a tick Icon or not
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/folder_eye.png"));
            } else {
                /// it's an isolated point
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(pointIndexes.second)->setIcon(QIcon(":/icons/eye_point.png"));
            }
        }
    } else {
        qDebug() << "MainWindow::displayPointMapEvent the pointView you are trying to show/hide is NULL";
    }
}

/**
 * @brief MainWindow::displayPointsInGroup
 * called when a user clicks the "eye" button in the Points menu
 */
void MainWindow::displayPointsInGroup(void){
    qDebug() << "MainWindow::displayPointsInGroup called";
    /// uncheck the other buttons

    pointsLeftWidget->getActionButtons()->getPlusButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getMinusButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getEditButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getMapButton()->setChecked(false);

    /// we hide those in case the previous button clicked was the plus button
    pointsLeftWidget->getGroupNameEdit()->hide();
    pointsLeftWidget->getGroupNameLabel()->hide();

    /// retrieves the id of the checked button within the group of buttons
    QString checkedName = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton()->text();

    /// it's a group
    if(points->isAGroup(checkedName)){
       pointsLeftWidget->setLastCheckedId(checkedName);
       pointsLeftWidget->getActionButtons()->getGoButton()->setChecked(false);
       pointsLeftWidget->hide();

       /// before we display the group of points, we make sure that the graphical object is consistent with the model
       DisplaySelectedGroup* selectedGroup = leftMenu->getDisplaySelectedGroup();
       leftMenu->updateGroupDisplayed(checkedName);
       selectedGroup->getPointButtonGroup()->setCheckable(true);
       selectedGroup->show();
       selectedGroup->setName(checkedName);

       switchFocus(checkedName, selectedGroup, MainWindow::WidgetType::GROUP);
       setMessageTop(TEXT_COLOR_INFO, "Click the map to add a permanent point");

    } else if(points->isAPoint(checkedName)){
        /// it's an isolated point
        DisplaySelectedPoint* selectedPoint = leftMenu->getDisplaySelectedPoint();
        QSharedPointer<PointView> pointView = points->findPointView(checkedName);
        QString robotName = "";

        if(pointView && pointView->getPoint()->isHome()){
            QPointer<RobotView> rv = robots->findRobotUsingHome(checkedName);
            if(rv != NULL)
                robotName = rv->getRobot()->getName();
            else
                qDebug() << "MainWindow::displayPointsInGroup something unexpected happened";
        }

        selectedPoint->setPointView(pointView, robotName);
        selectedPoint->displayPointInfo();
        selectedPoint->show();

        if(pointView->isVisible())
            selectedPoint->getActionButtons()->getMapButton()->setChecked(true);
        else
            selectedPoint->getActionButtons()->getMapButton()->setChecked(false);
        switchFocus("Point", selectedPoint, MainWindow::WidgetType::POINT);
        pointsLeftWidget->getActionButtons()->getGoButton()->setChecked(false);
        pointsLeftWidget->hide();
    }
}

/**
 * @brief MainWindow::removePointFromInformationMenu
 * called when a user clicks a point on the map and then tries to remove it clicking the "minus" button
 */
void MainWindow::removePointFromInformationMenu(void){
    qDebug() << "removepointfrominformationmenu event called";
     int ret = openConfirmMessage("Are you sure you want to remove this point ?");
    leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMinusButton()->setChecked(false);
    switch(ret){
        case QMessageBox::Cancel :
            qDebug() << "clicked no";
        break;
        case QMessageBox::Ok : {
            /// first we check that this point is not a home
            QSharedPointer<PointView> pointView = points->findPointView(leftMenu->getDisplaySelectedPoint()->getPointName());
            if(pointView && !pointView->getPoint()->isHome()){
                QString pointName = pointView->getPoint()->getName();

                /// holds the index of the group and the index of a particular point in this group within <points>
                QPair<QString, int> pointIndexes = points->findPointIndexes(pointName);

                if(pointIndexes.first.compare("")!= 0){

                    /// it's an isolated point
                    if(points->isAPoint(pointIndexes.first)){
                        /// need to remove the point from the map
                        pointView->hide();

                        /// updates the model
                        points->removePoint(pointName);

                        /// updates the file containing containing points info
                        savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

                        /// updates the list of points
                        pointsLeftWidget->getGroupButtonGroup()->updateButtons();
                        backEvent();

                    } else {
                        /// need to remove the point from the map
                        pointView->hide();

                        /// updates the model
                        points->removePoint(pointName);

                        /// updates the file containing containing points info
                        savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

                        /// updates the group menu
                        leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->setGroup(pointsLeftWidget->getLastCheckedId());

                        /// closes the window
                        backEvent();

                        /// if the group is empty the user is asked whether or not he wants to delete it
                        if(points->findGroup(pointIndexes.first)->isEmpty() && pointIndexes.first.compare(NO_GROUP_NAME) != 0){
                            int res = openEmptyGroupMessage(pointIndexes.first);

                            /// the group must be deleted
                            if(res == QMessageBox::Yes){
                                /// updates model
                                points->removeGroup(pointIndexes.first);

                                /// updates file
                                savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

                                /// updates menu
                                pointsLeftWidget->getGroupButtonGroup()->updateButtons();
                                createPointWidget->updateGroupBox();
                                backEvent();
                            }
                        }
                        updateAllPaths(*pointView->getPoint(), Point());
                        setTemporaryMessageTop(TEXT_COLOR_SUCCESS, "You have deleted the point : " + pointName + " from the group : " + pointIndexes.first, 2500);
                    }
                } else {
                    qDebug() << "could not find this point";
                }
            } else {
                /// this point is actually the home point of a robot and therefore cannot be removed
                QPointer<RobotView> robot = robots->findRobotUsingHome(pointView->getPoint()->getName());
                if(robot != NULL){
                    openInterdictionOfPointRemovalMessage(pointView->getPoint()->getName(), robot->getRobot()->getName());
                    qDebug() << "Sorry this point is the home of a robot and therefore cannot be removed";
                } else {
                    qDebug() << "removePointFromInformationMenu : something unexpected happened";
                }
            }
        }
        break;
        default:
            Q_UNREACHABLE();
        /// should never be here
            qDebug() << "MainWindow::removePointFromInformationMenu should not be here";
        break;
    }
}

/**
 * @brief MainWindow::editPointFromGroupMenu
 */
void MainWindow::editPointFromGroupMenu(void){
    qDebug() << "editgroupfrommenuevent";
    setMessageTop(TEXT_COLOR_INFO, "Click the map or drag the point to change its position");
    QString groupName = leftMenu->getDisplaySelectedGroup()->getNameLabel()->text();

    qDebug() << "working on group" << groupName << "and id" << leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->checkedId();

    QString pointName = static_cast<CustomPushButton *> (leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->checkedButton())->text();

    if(pointName.compare("") != 0){
        /// update the pointview and show the point on the map with hover color
        QString robotName = "";
        if(points->findPointView(pointName)->getPoint()->isHome()){
            QPointer<RobotView> rv = robots->findRobotUsingHome(pointName);
            if(rv != NULL)
                robotName = rv->getRobot()->getName();
            else
                qDebug() << "editPointFromGroupMenu : something unexpected happened";
        }
        qDebug() << "name point u trying to edit" << pointName;
        QSharedPointer<PointView> displaySelectedPointView = points->findPointView(pointName);
        displaySelectedPointView->setOriginalPosition(displaySelectedPointView->getPoint()->getPosition());

        if(displaySelectedPointView){
            qDebug() << "about to put u orange";

            leftMenu->getDisplaySelectedPoint()->setPointView(displaySelectedPointView, robotName);

            displaySelectedPointView->setPixmap(PointView::PixmapType::SELECTED);
            displaySelectedPointView->show();

            /// update the file
            savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

            /// sets the state of the map and the other widgets to prevent other concurrent actions
            setGraphicItemsState(GraphicItemState::NO_EVENT);
            mapPixmapItem->setState(GraphicItemState::EDITING_PERM);

            /// sets the state of the point of the map to make it draggable
            displaySelectedPointView->setState(GraphicItemState::EDITING_PERM);
            displaySelectedPointView->setFlag(QGraphicsItem::ItemIsMovable, true);

            leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMinusButton()->setEnabled(false);
            leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMinusButton()->setToolTip("");
            leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMapButton()->setEnabled(false);
            leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMapButton()->setToolTip("");
            leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMapButton()->setChecked(true);

            /// to force the user to click either the save or the cancel button
            leftMenu->getDisplaySelectedPoint()->getActionButtons()->getEditButton()->setEnabled(false);
            leftMenu->getDisplaySelectedPoint()->getActionButtons()->getEditButton()->setToolTip("You can choose to save or discard your modifications by clicking the save (Enter) and cancel button respectively");

            leftMenu->getDisplaySelectedPoint()->displayPointInfo();

            leftMenu->getDisplaySelectedPoint()->getActionButtons()->getEditButton()->setChecked(true);
            leftMenu->getDisplaySelectedPoint()->getNameEdit()->setText("");
            leftMenu->getDisplaySelectedPoint()->getNameEdit()->setPlaceholderText(pointName);
            leftMenu->getDisplaySelectedPoint()->getCancelButton()->show();
            leftMenu->getDisplaySelectedPoint()->getSaveButton()->show();
            leftMenu->getDisplaySelectedPoint()->getNameEdit()->show();
            leftMenu->getDisplaySelectedPoint()->getNameEdit()->setFocus();
            leftMenu->getDisplaySelectedPoint()->getNameLabel()->hide();
            leftMenu->getDisplaySelectedPoint()->show();
            leftMenu->getDisplaySelectedGroup()->hide();
            switchFocus(leftMenu->getDisplaySelectedPoint()->getPointName(),leftMenu->getDisplaySelectedPoint(), MainWindow::WidgetType::POINT);
        }
    }
}

/**
 * @brief MainWindow::displayPointInfoFromGroupMenu
 * display the information of a point from the group menu
 */
void MainWindow::displayPointInfoFromGroupMenu(void){
    qDebug() << "display point info from group menu event called";
    /// retrieves a pointer to the pointView using the text of the label

    QString pointName = static_cast<CustomPushButton*>(leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->checkedButton())->text();
    QSharedPointer<PointView> pointView = points->findPointView(pointName);


    if(pointName.compare("") != 0 && pointView){
        setMessageTop(TEXT_COLOR_NORMAL, "");
        DisplaySelectedPoint* selectedPoint = leftMenu->getDisplaySelectedPoint();

        QString robotName = "";
        if(pointView->getPoint()->isHome()){
            QPointer<RobotView> rv = robots->findRobotUsingHome(pointName);
            if(rv != NULL)
                robotName = rv->getRobot()->getName();
            else
                qDebug() << "setSelectedRobotFromPoint : something unexpected happened";
        }

        selectedPoint->setPointView(pointView, robotName);
        selectedPoint->displayPointInfo();

        /// map is checked if the point is displayed
        if(pointView->isVisible())
            selectedPoint->getActionButtons()->getMapButton()->setChecked(true);
        else
            selectedPoint->getActionButtons()->getMapButton()->setChecked(false);

        selectedPoint->show();
        leftMenu->getDisplaySelectedGroup()->hide();
        switchFocus(selectedPoint->getPointName(), selectedPoint, MainWindow::WidgetType::POINT);
    } else {
        qDebug() << "no point named :" << pointName;
    }
}

/**
 * @brief MainWindow::updatePoint
 * called when a user edits a point and save the changes either by pressing the enter key or clicking the save button
 */
void MainWindow::updatePoint(void){

    qDebug() << "MainWindow::updatePoint";
    DisplaySelectedPoint* selectedPoint = leftMenu->getDisplaySelectedPoint();
    QSharedPointer<PointView> displaySelectedPointView = points->findPointView(selectedPoint->getPointName());
    /// to update the paths
    Point copy = *displaySelectedPointView->getPoint();
    /// the position of the point before edition, it is needed to give the paths the old point (name and pos) and the new point
    copy.setPosition(displaySelectedPointView->getOriginalPosition().getX(),
                     displaySelectedPointView->getOriginalPosition().getY());

    /// if it is a valid point on the map
    if(map->getMapImage().pixelColor(displaySelectedPointView->getPoint()->getPosition().getX(),
                                     displaySelectedPointView->getPoint()->getPosition().getY()).red() >= 254){

        ///resets the tooltip of the edit button and the minus button
        leftMenu->getDisplaySelectedPoint()->getActionButtons()->getEditButton()->setToolTip("Click here and then choose between clicking on the map or drag the point to change its position");
        leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMinusButton()->setToolTip("Click here to remove the point");

        leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMapButton()->setEnabled(true);
        leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMapButton()->setToolTip("Click to hide this point");

        /// resets the color of the pointView
        points->getTmpPointView()->setPixmap(PointView::PixmapType::NORMAL);

        /// notifies the map that the point's name has changed and that the hover has to be updated
        /// if the field has been left empty we keep the old name
        if(leftMenu->getDisplaySelectedPoint()->getNameEdit()->text().simplified().compare("")){
            emit nameChanged(displaySelectedPointView->getPoint()->getName(), leftMenu->getDisplaySelectedPoint()->getNameLabel()->text());
            leftMenu->getDisplaySelectedPoint()->getNameLabel()->setText(leftMenu->getDisplaySelectedPoint()->getNameEdit()->text().simplified());
        }
        else
            emit nameChanged(displaySelectedPointView->getPoint()->getName(), displaySelectedPointView->getPoint()->getName());

        /// updates the name in the label
        if(leftMenu->getDisplaySelectedPoint()->getNameEdit()->text().simplified().compare(""))
            displaySelectedPointView->getPoint()->setName(leftMenu->getDisplaySelectedPoint()->getNameEdit()->text());

        /// updates the position of the point
        /// to determine wheter the coordinate is 2 digits long or 3 digits long in order to parse them correctly
        int xLength = leftMenu->getDisplaySelectedPoint()->getXLabel()->text().count();
        int yLength = leftMenu->getDisplaySelectedPoint()->getYLabel()->text().count();

        displaySelectedPointView->setPos(
        leftMenu->getDisplaySelectedPoint()->getXLabel()->text().right(xLength-4).toFloat(),
        leftMenu->getDisplaySelectedPoint()->getYLabel()->text().right(yLength-4).toFloat());


        /// save changes to the file
        savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

        /// if the point is the home of a robot, we update the file containing the home on the robot
        // TODO rework home
        qDebug() << "MainWindow::updatePoint need to update if home";
        /*if(displaySelectedPointView->getPoint()->isHome()){
            QPointer<RobotView> robotView = robots->getRobotViewByName(displaySelectedPointView->getPoint()->getRobotName());

            if(robotView && robotView->getRobot()->commandController->sendCommand(QString("n \"") + QString::number(displaySelectedPointView->getPoint()->getPosition().getX()) + "\" \""
                                                              + QString::number(displaySelectedPointView->getPoint()->getPosition().getY()) + "\"")){
                QString answerHome = robotView->getRobot()->waitAnswer();
                QStringList answerList2 = answerHome.split(QRegExp("[ ]"), QString::SkipEmptyParts);
                if(answerList2.size() > 1){
                    QString cmd2 = answerList2.at(0);
                    bool success2 = (answerList2.at(1).compare("done") == 0);
                    if((cmd2.compare("n") == 0 && success2) || answerList2.at(0).compare("1") == 0){
                        setMessageTop(TEXT_COLOR_SUCCESS, robotView->getRobot()->getName() + " successfully saved its home point");
                    } else
                        setMessageTop(TEXT_COLOR_DANGER, robotView->getRobot()->getName() + " failed to save its home point, please try again");
                }
            } else {
                qDebug() << "MainWindow::updatePoint could not find the robot named" << displaySelectedPointView->getPoint()->getRobotName();
            }
        }*/

        /// so that you cannot edit a new name unless you click the edit button again
        selectedPoint->getActionButtons()->getEditButton()->setChecked(false);

        /// we hide the save button and the cancel button
        selectedPoint->getCancelButton()->hide();
        selectedPoint->getSaveButton()->hide();

        /// reset the state of the map so we can click it again
        setGraphicItemsState(GraphicItemState::NO_STATE);

        /// enable the edit button and the minus button again
        leftMenu->getDisplaySelectedPoint()->getActionButtons()->getEditButton()->setEnabled(true);
        leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMinusButton()->setEnabled(true);

        /// updates the isolated points in the group menus
        pointsLeftWidget->getGroupButtonGroup()->updateButtons();

        /// we enable the "back" button again
        leftMenu->getReturnButton()->setEnabled(true);
        leftMenu->getReturnButton()->setToolTip("");

        /// We update the path as this point might have been used in a path
        updateAllPaths(copy, *displaySelectedPointView->getPoint());
        qDebug() << "copy" << copy.getName() << copy.getPosition().getX() << copy.getPosition().getY();
        qDebug() << " new" << displaySelectedPointView->getPoint()->getName() << displaySelectedPointView->getPoint()->getPosition().getX() << displaySelectedPointView->getPoint()->getPosition().getY();

        leftMenu->getDisplaySelectedPoint()->getNameEdit()->hide();
        leftMenu->getDisplaySelectedPoint()->getNameLabel()->show();

        topLayout->setLabelDelay(TEXT_COLOR_SUCCESS, "Your point has been successfully updated", 4000);
    } /// otherwise the point is not saved and an error message is displayed at the top
    else
        setTemporaryMessageTop(TEXT_COLOR_DANGER, "You cannot save your point \"" +
                               displaySelectedPointView->getPoint()->getName() +
                               "\" because this area of the map is not known to your robot(s), as a result your robot(s) would not be able to move there", 3500);
}

/**
 * @brief MainWindow::cancelEvent
 * called when a user discard the changes made about a point
 */
void MainWindow::cancelEvent(void){
    qDebug() << "cancel edit point event called";
    leftMenu->getDisplaySelectedPoint()->getNameEdit()->hide();
    leftMenu->getDisplaySelectedPoint()->getNameLabel()->show();
    leftMenu->getCloseButton()->setEnabled(true);
    leftMenu->getReturnButton()->setEnabled(true);
    topLayout->setEnabled(true);
    /// reset the color of the pointView
    QSharedPointer<PointView> displaySelectedPointView = points->findPointView(leftMenu->getDisplaySelectedPoint()->getPointName());
    if(displaySelectedPointView){
        displaySelectedPointView->setPixmap(PointView::PixmapType::NORMAL);
        points->getTmpPointView()->setPixmap(PointView::PixmapType::NORMAL);
        qDebug() << "about to reset your position";

        displaySelectedPointView->getPoint()->setPosition(displaySelectedPointView->getOriginalPosition());

        /// we hide the buttons relative to the edit option and make sure the points properties are not longer modifiable
        leftMenu->getDisplaySelectedPoint()->getActionButtons()->getEditButton()->setChecked(false);
        leftMenu->getDisplaySelectedPoint()->getCancelButton()->hide();
        leftMenu->getDisplaySelectedPoint()->getSaveButton()->hide();

        /// in case the user had dragged the point around the map or clicked it, this resets the coordinates displayed to the original ones
        leftMenu->getDisplaySelectedPoint()->getXLabel()->setText(QString("X : ") + QString::number(
                                                                      displaySelectedPointView->getPoint()->getPosition().getX()));
        leftMenu->getDisplaySelectedPoint()->getYLabel()->setText(QString("Y : ") + QString::number(
                                                                      displaySelectedPointView->getPoint()->getPosition().getY()));
        /// enable the edit button again and the map button
        leftMenu->getDisplaySelectedPoint()->getActionButtons()->getEditButton()->setEnabled(true);
        leftMenu->getDisplaySelectedPoint()->getActionButtons()->getEditButton()->setToolTip("You can click on this button and then choose between clicking on the map or drag the point to change its position");
        leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMapButton()->setEnabled(true);
        leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMapButton()->setToolTip("Click to hide this point");

        /// resets the tooltip of the minus button
        leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMinusButton()->setToolTip("You can click this button to remove the point");
        leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMinusButton()->setEnabled(true);

        /// reset the state of the map so we can click it again
        setGraphicItemsState(GraphicItemState::NO_STATE);
        displaySelectedPointView->setPos(static_cast<qreal>(displaySelectedPointView->getPoint()->getPosition().getX()),
                                                                    static_cast<qreal>(displaySelectedPointView->getPoint()->getPosition().getY()));
        /// reset its name in the hover on the map
        leftMenu->getDisplaySelectedPoint()->getNameEdit()->setText(displaySelectedPointView->getPoint()->getName());
    } else {
        qDebug() << "MainWindow::cancelEvent can't find the pointView :" << leftMenu->getDisplaySelectedPoint()->getPointName();
    }
    /// enable the back button in case we were editing coming from the left menu
    leftMenu->getReturnButton()->setEnabled(true);
    leftMenu->getReturnButton()->setToolTip("");

    setMessageTop(TEXT_COLOR_INFO, "Your point \"" + leftMenu->getDisplaySelectedPoint()->getPointName() + "\" has not been modified");
}

/**
 * @brief MainWindow::updateCoordinates
 * @param x
 * @param y
 * updates the coordinates of the selected point
 */
void MainWindow::updateCoordinates(double x, double y){
    qDebug() << "updateCoordinates called";
    leftMenu->getDisplaySelectedPoint()->getXLabel()->setText("X : " + QString::number(x, 'f', 1));
    leftMenu->getDisplaySelectedPoint()->getYLabel()->setText("Y : " + QString::number(y, 'f', 1));
    points->findPointView(leftMenu->getDisplaySelectedPoint()->getPointName())->setPos(x, y);

    if(map->getMapImage().pixelColor(x ,y).red() >= 254){
        leftMenu->getDisplaySelectedPoint()->getSaveButton()->setEnabled(true);
        setMessageTop(TEXT_COLOR_INFO, "To save this point permanently click \"Save\" or press Enter");
    }
    else {
        leftMenu->getDisplaySelectedPoint()->getSaveButton()->setEnabled(false);
        setMessageTop(TEXT_COLOR_WARNING, "You cannot save this point because the current position is known as an obstacle for the robot");
    }
}

/**
 * @brief MainWindow::removePointFromGroupMenu
 * removes a point which does not belong to the default group, from the group menu
 */
void MainWindow::removePointFromGroupMenu(void){
    QString checkedId = static_cast<CustomPushButton *> (leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->checkedButton())->text();

    if(checkedId.compare("") != 0)
        askForDeletePointConfirmation(checkedId);
    else
        qDebug() << "can't remove point without name";
}

/**
 * @brief MainWindow::displayPointFromGroupMenu
 * called when a user displays or hides a point on the map from the group menu
 */
void MainWindow::displayPointFromGroupMenu(){

    const QString pointName = leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->checkedButton()->text();

    qDebug() << "displaypointfromgroupmenu event called" << pointName ;

    int checkedId = leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonIdByName(pointName);

    if(checkedId != -1){
        QSharedPointer<PointView> currentPointView = points->findPointView(pointName);

        /// if the point is displayed we stop displaying it
        if(currentPointView->isVisible()){

            /// hides the point on the map
            currentPointView->hide();

            /// removes the tick icon to show that the point is not displayed on the map
            leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->buttons()[checkedId]->setIcon(QIcon(":/icons/space_point.png"));

            /// updates the file
            savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

            /// if the entire group was displayed it is not the case anymore
            if(pointsLeftWidget->getGroupButtonGroup()->getButtonByName(pointsLeftWidget->getLastCheckedId()) != NULL)
                pointsLeftWidget->getGroupButtonGroup()->getButtonByName(pointsLeftWidget->getLastCheckedId())->setIcon(QIcon(":/icons/folder_space.png"));

            /// changes the map button message
            leftMenu->getDisplaySelectedGroup()->getActionButtons()->getMapButton()->setToolTip("Click to display the selected point on the map");

        } else {

            /// shows the point on the map
            currentPointView->show();

            /// we add a tick icon next to the name of the point to show that it is displayed on the map
            leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->buttons()[checkedId]->setIcon(QIcon(":/icons/eye_point.png"));

            /// saves changes to the file
            savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

            /// we check whether or not the entire group is displayed and update the points left widget accordingly by adding a tick Icon or not
            if(points->isDisplayed(pointsLeftWidget->getLastCheckedId())){
                if(pointsLeftWidget->getGroupButtonGroup()->getButtonByName(pointsLeftWidget->getLastCheckedId()) != NULL)
                    pointsLeftWidget->getGroupButtonGroup()->getButtonByName(pointsLeftWidget->getLastCheckedId())->setIcon(QIcon(":/icons/folder_eye.png"));
            }

            /// changes the map button message
            leftMenu->getDisplaySelectedGroup()->getActionButtons()->getMapButton()->setToolTip("Click to hide the selected point on the map");

        }
    } else {
        /// should never be here
        qDebug() << "can't handle a point with index -1";
    }
}

/**
 * @brief MainWindow::openInterdictionOfPointRemovalMessage
 * @param pointName
 * @param robotName
 * opens a message box to notify a user that the point he is trying to remove cannot
 * be removed because it is the home point of a robot
 */
void MainWindow::openInterdictionOfPointRemovalMessage(const QString pointName, const QString robotName){
    QMessageBox msgBox;
    msgBox.setText("The point : " + pointName + " that you are trying to remove is the home point of the robot " + robotName +
                   ". If you want to remove it you first have to indicate a new home point for this robot.");
    msgBox.setIcon(QMessageBox::Critical);
    msgBox.setStandardButtons(QMessageBox::Ok);
    msgBox.setInformativeText("To modify the home point of a robot you can either click on the menu > Robots, choose a robot and Add Home or simply click a robot on the map and Add Home");
    msgBox.exec();
}

/**
 * @brief MainWindow::doubleClickOnRobot
 * @param id
 * does the same as clicking on a robot and then on the eye button
 */
void MainWindow::doubleClickOnRobot(QString id){
    qDebug() << "double click on robot" << id;
    setSelectedRobot(robots->getRobotViewByName(id));
}

/**
 * @brief MainWindow::doubleClickOnPoint
 * @param pointName
 * does the same as clicking on a point and then on the eye button
 */
void MainWindow::doubleClickOnPoint(QString pointName){
    qDebug() << "MainWindow::double click on point";
    setMessageTop(TEXT_COLOR_NORMAL, "");
    QSharedPointer<PointView> pointView = points->findPointView(pointName);

    if(pointView){
        DisplaySelectedPoint* selectedPoint = leftMenu->getDisplaySelectedPoint();
        QString robotName = "";
        if(pointView->getPoint()->isHome()){
            QPointer<RobotView> rv = robots->findRobotUsingHome(pointName);
            if(rv != NULL)
                robotName = rv->getRobot()->getName();
            else
                qDebug() << "doubleClickOnPoint : something unexpected happened";
        }
        selectedPoint->setPointView(pointView, robotName);
        selectedPoint->displayPointInfo();

        if(pointView->isVisible())
            selectedPoint->getActionButtons()->getMapButton()->setChecked(true);
        else
            selectedPoint->getActionButtons()->getMapButton()->setChecked(false);
        selectedPoint->show();
        leftMenu->getDisplaySelectedGroup()->hide();
        switchFocus(selectedPoint->getPointName(), selectedPoint, MainWindow::WidgetType::POINT);
    } else {
        qDebug()  << "no group " << leftMenu->getDisplaySelectedGroup()->getNameLabel()->text();
    }

}

/**
 * @brief MainWindow::doubleClickOnGroup
 * @param checkedId
 * does the same as clicking on a group or a point belonging to the default group
 * and then on the eye button
 */
void MainWindow::doubleClickOnGroup(QString checkedName){
    qDebug() << "double click on group or default point " << checkedName;
    /// uncheck the other buttons

    pointsLeftWidget->getActionButtons()->getPlusButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getMinusButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getEditButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getMapButton()->setChecked(false);

    /// we hide those in case the previous button clicked was the plus button
    pointsLeftWidget->getGroupNameEdit()->hide();
    pointsLeftWidget->getGroupNameLabel()->hide();

    /// it's a group
    if(points->isAGroup(checkedName)){
       setMessageTop(TEXT_COLOR_INFO, "Click the map to add a permanent point");
       pointsLeftWidget->setLastCheckedId(checkedName);
       pointsLeftWidget->getActionButtons()->getGoButton()->setChecked(false);
       pointsLeftWidget->hide();

       /// before we display the group of points, we make sure that the graphical object is consistent with the model
       DisplaySelectedGroup* selectedGroup = leftMenu->getDisplaySelectedGroup();
       leftMenu->updateGroupDisplayed(checkedName);
       selectedGroup->getPointButtonGroup()->setCheckable(true);
       selectedGroup->show();
       selectedGroup->setName(checkedName);

       switchFocus(checkedName, selectedGroup, MainWindow::WidgetType::GROUP);
    } else if(points->isAPoint(checkedName)){

        /// it's an isolated point
        setMessageTop(TEXT_COLOR_NORMAL, "");
        DisplaySelectedPoint* selectedPoint = leftMenu->getDisplaySelectedPoint();
        QSharedPointer<PointView> pointView = points->findPointView(checkedName);

        QString robotName = "";
        if(pointView->getPoint()->isHome()){
            QPointer<RobotView> rv = robots->findRobotUsingHome(pointView->getPoint()->getName());
            if(rv != NULL)
                robotName = rv->getRobot()->getName();
            else
                qDebug() << "doubleClickOnGroup : something unexpected happened";
        }

        selectedPoint->setPointView(pointView, robotName);
        selectedPoint->displayPointInfo();
        selectedPoint->show();

        pointsLeftWidget->getActionButtons()->getGoButton()->setChecked(false);
        if(pointView->isVisible())
            selectedPoint->getActionButtons()->getMapButton()->setChecked(true);
        else
            selectedPoint->getActionButtons()->getMapButton()->setChecked(false);

        pointsLeftWidget->hide();
        switchFocus(checkedName, selectedPoint, MainWindow::WidgetType::POINT);
    }
}

/**
 * @brief MainWindow::reestablishConnections
 * to reestablish the double clicks after points are updated (because buttons in the menu are recreated)
 */
void MainWindow::reestablishConnectionsGroups(){
    qDebug() << " connections for groups requested";
    foreach(QAbstractButton* button, pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->buttons())
        connect(button, SIGNAL(doubleClick(QString)), this, SLOT(doubleClickOnGroup(QString)));
}

/**
 * @brief MainWindow::reestablishConnections
 * to reestablish the double clicks after groups are updated
 */
void MainWindow::reestablishConnectionsPoints(){
    qDebug() << "connections for points requested";
    foreach(QAbstractButton* button, leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->buttons())
        connect(button, SIGNAL(doubleClick(QString)), this, SLOT(doubleClickOnPoint(QString)));
}

/**
 * @brief MainWindow::openEmptyGroupMessage
 * @param groupName
 * @return int
 * To ask a user if he wants to delete a group after deleting its last point
 */
int MainWindow::openEmptyGroupMessage(const QString groupName){
    QMessageBox msgBox;
    msgBox.setText("The group " + groupName + " is empty. Do you want to delete this group permanently ?");
    msgBox.setStandardButtons(QMessageBox::No | QMessageBox::Yes);
    msgBox.setDefaultButton(QMessageBox::No);
    return msgBox.exec();
}

void MainWindow::createGroup(QString groupName){
    qDebug() << "createGroup called" << groupName;

    groupName = groupName.simplified();
    if(pointsLeftWidget->checkGroupName(groupName) == 0){
        pointsLeftWidget->setLastCheckedId("");

        /// updates the model
        points->addGroup(groupName);

        /// updates the file
        savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

        /// updates list of groups in menu
        pointsLeftWidget->updateGroupButtonGroup();

        /// updates the comboBox to make this new group available when a user creates a point
        createPointWidget->updateGroupBox();

        /// enables the return button again
        leftMenu->getReturnButton()->setEnabled(true);

        /// hides everything that's related to the creation of a group
        pointsLeftWidget->getCancelButton()->hide();
        pointsLeftWidget->getSaveButton()->hide();
        pointsLeftWidget->getGroupNameEdit()->hide();
        pointsLeftWidget->getGroupNameLabel()->hide();

        /// enables the plus button again
        pointsLeftWidget->disableButtons();
        pointsLeftWidget->getActionButtons()->getPlusButton()->setEnabled(true);
        pointsLeftWidget->getActionButtons()->getPlusButton()->setToolTip("Click here to add a new group");
        topLayout->setEnabled(true);

        topLayout->setLabelDelay(TEXT_COLOR_SUCCESS, "You have successfully created a new group : \"" + groupName + "\"", 4000);
    } else if(pointsLeftWidget->checkGroupName(groupName) == 1){
        pointsLeftWidget->setLastCheckedId("");

        /// updates list of groups in menu
        pointsLeftWidget->updateGroupButtonGroup();

        /// enables the return button again
        leftMenu->getReturnButton()->setEnabled(true);

        /// hides everything that's related to the creation of a group
        pointsLeftWidget->getCancelButton()->hide();
        pointsLeftWidget->getSaveButton()->hide();
        pointsLeftWidget->getGroupNameEdit()->hide();
        pointsLeftWidget->getGroupNameLabel()->hide();

        /// enables the plus button again
        pointsLeftWidget->disableButtons();
        pointsLeftWidget->getActionButtons()->getPlusButton()->setEnabled(true);
        pointsLeftWidget->getActionButtons()->getPlusButton()->setToolTip("Click here to add a new group");
        topLayout->setEnabled(true);
    } else
        topLayout->setLabelDelay(TEXT_COLOR_DANGER, "You cannot choose : " + groupName + " as a new name for your group because another group already has this name", 4000);
}

void MainWindow::modifyGroupWithEnter(QString name){
    name = name.simplified();
    qDebug() << "MainWindow::modifyGroupWithEnter called : modifying group after enter key pressed from" << pointsLeftWidget->getLastCheckedId() << "to" << name;

    topLayout->setEnabled(true);
    setEnableAll(true);
    QString oldGroupName = pointsLeftWidget->getLastCheckedId();
    qDebug() << "checkgroupname result is" << pointsLeftWidget->checkGroupName(name);
    if(pointsLeftWidget->checkGroupName(name) == 0){
        qDebug() << "this name is ok";

        /// Updates the model
        qDebug() << pointsLeftWidget->getLastCheckedId();
        points->getGroups()->insert(name, points->getGroups()->value(pointsLeftWidget->getLastCheckedId()));
        qDebug() << "I have removed " << points->getGroups()->remove(pointsLeftWidget->getLastCheckedId()) << "item(s)";

        /// updates the group box to create a point
        createPointWidget->updateGroupBox();

        /// enables the plus button
        pointsLeftWidget->getActionButtons()->getPlusButton()->setEnabled(true);

        /// saves to file
        savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

        /// enables the buttons
        pointsLeftWidget->getGroupButtonGroup()->setEnabled(true);
        pointsLeftWidget->disableButtons();

        /// updates view
        int checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonIdByName(pointsLeftWidget->getGroupButtonGroup()->getEditedGroupName());

        pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->show();
        static_cast<CustomPushButton*> (pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId))->setText(name);
        pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->hide();

        pointsLeftWidget->setLastCheckedId("");

        /// resets the pixmaps
        points->setPixmapAll(PointView::PixmapType::NORMAL);

        topLayout->setLabelDelay(TEXT_COLOR_SUCCESS, "You have successfully changed the name of your group from \"" + oldGroupName + "\" to \"" + name + "\"", 4000);

    } else if(pointsLeftWidget->checkGroupName(name) == 1){
        /// enables the buttons
        pointsLeftWidget->getGroupButtonGroup()->setEnabled(true);
        pointsLeftWidget->disableButtons();

        /// updates view
        int checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonIdByName(pointsLeftWidget->getGroupButtonGroup()->getEditedGroupName());

        pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->show();
        pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->hide();

        /// enables the plus button
        pointsLeftWidget->getActionButtons()->getPlusButton()->setEnabled(true);

        /// resets the pixmaps
        points->setPixmapAll(PointView::PixmapType::NORMAL);

        pointsLeftWidget->setLastCheckedId("");
    }
    else
        topLayout->setLabelDelay(TEXT_COLOR_DANGER, "You cannot choose : " + name + " as a new name for your group because another group already has this name", 4000);
}

void MainWindow::modifyGroupAfterClick(QString name){
    name = name.simplified();
    qDebug() << "modifyGroupAfterClick called from" << pointsLeftWidget->getLastCheckedId() << "to" << name;
    topLayout->setEnabled(true);

    if (pointsLeftWidget->getLastCheckedId() != "")
     {
        /// resets the menu
        pointsLeftWidget->getActionButtons()->getPlusButton()->setEnabled(true);
        pointsLeftWidget->getGroupButtonGroup()->setEnabled(true);
        pointsLeftWidget->disableButtons();
        pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->hide();

        int checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonIdByName(pointsLeftWidget->getGroupButtonGroup()->getEditedGroupName());
           QString color ="";
           QString msg = "";
        if(pointsLeftWidget->checkGroupName(name) == 0){
            /// Update the model
            qDebug() << pointsLeftWidget->getLastCheckedId();
            points->getGroups()->insert(name, points->getGroups()->take(pointsLeftWidget->getLastCheckedId()));

            /// saves to file
            savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

            color = TEXT_COLOR_SUCCESS;
            msg = "You have successfully modified the name of your group";
        } else if(pointsLeftWidget->checkGroupName(name) == 1){
            color = TEXT_COLOR_DANGER;
            msg = "The name of your group cannot be empty. Please choose a name for your group";
        } else {
            color = TEXT_COLOR_DANGER;
            msg = "You cannot choose : " + name.simplified() + " as a new name for your group because another group already has this name";
        }

        pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->show();
        pointsLeftWidget->setLastCheckedId("");
        topLayout->setLabelDelay(color, msg, 4000);
    }
}

void MainWindow::enableReturnAndCloseButtons(){
    leftMenu->getReturnButton()->setEnabled(true);
    leftMenu->getCloseButton()->setEnabled(true);
    topLayout->setEnabled(true);
}

void MainWindow::setMessageCreationGroup(QString type, QString message){
    setMessageTop(type, message);
}

void MainWindow::setMessageCreationPoint(QString type, CreatePointWidget::Error error){
    qDebug() << "setMessageCreation point called from mainwindow";
    switch(error){
    case CreatePointWidget::Error::NoError:
        setMessageTop(type, "Click save or press ENTER to save this point");
        break;
    case CreatePointWidget::Error::ContainsSemicolon:
        setMessageTop(type, "You cannot create a point with a name that contains a semicolon, a curly bracket or the pattern \"pathpoint\"");
        break;
    case CreatePointWidget::Error::EmptyName:
        setMessageTop(type, "You cannot create a point with an empty name");
        break;
    case CreatePointWidget::Error::AlreadyExists:
        setMessageTop(type, "You cannot create a point with this name because a point with the same name already exists");
        break;
    default:
        Q_UNREACHABLE();
        qDebug() << "Should never be here, if you do get here however, check that you have not added a new error code and forgotten to add it in the cases afterwards";
        break;
    }
}

void MainWindow::choosePointName(QString message){
    setMessageTop(TEXT_COLOR_INFO, message);
}

/**********************************************************************************************************************************/

//                                          PATHS

/**********************************************************************************************************************************/

void MainWindow::initializePaths(){
    deserializePaths(QDir::currentPath() + QDir::separator() + "paths.dat");
}

void MainWindow::serializePaths(const QString fileName){
    QFile pathFile(fileName);
    pathFile.resize(0);
    pathFile.open(QIODevice::ReadWrite);
    QDataStream out(&pathFile);
    out << *paths;
    pathFile.close();
}

void MainWindow::deserializePaths(const QString fileName){
    QFile pathFile(fileName);
    pathFile.open(QIODevice::ReadWrite);
    QDataStream in(&pathFile);
    Paths tmpPaths;
    in >> tmpPaths;
    pathFile.close();
    paths->setGroups(tmpPaths.getGroups());
}

void MainWindow::pathBtnEvent(){
    hideAllWidgets();
    robots->deselect();
    bottomLayout->uncheckRobots();
    bottomLayout->setLastCheckedId(-1);
    /// resets the list of groups menu
    switchFocus("Paths", leftMenu->getGroupsPathsWidget(), MainWindow::WidgetType::GROUPS_PATHS);
    leftMenu->getGroupsPathsWidget()->show();
    /*
    qDebug() << "pointBtnEvent called ";
    /// we uncheck all buttons from all menus
    leftMenu->getDisplaySelectedGroup()->uncheck();
    hideAllWidgets();
    pointsLeftWidget->show();
    setMessageTop(TEXT_COLOR_INFO, "Click the map to add a permanent point");*/
}

void MainWindow::deletePathSlot(QString groupName, QString pathName){
    qDebug() << "MainWindow::deletePathSlot called on group :" << groupName << ", path :" << pathName;
    int answer = openConfirmMessage("Are you sure you want to delete this path, this action is irreversible ?");
    switch(answer){
    case QMessageBox::StandardButton::Ok:
    {
        paths->deletePath(groupName, pathName);
        serializePaths(QDir::currentPath() + QDir::separator() + "paths.dat");

        leftMenu->getPathGroupDisplayed()->setPathsGroup(groupName);
        leftMenu->getPathGroupDisplayed()->show();
        leftMenu->getDisplaySelectedPath()->hide();
        if(!pathPainter->getVisiblePath().compare(pathName)){
            qDebug() << "hey i have to stop displaying this path that was destroyed";
            emit resetPath();
        } else
            qDebug() << "dont have to stop displaying this path" <<
                        pathPainter->getVisiblePath() <<
                        "because the last one checked is"
                     << leftMenu->getPathGroupDisplayed()->getLastCheckedButton();
        backEvent();
    }
    break;
    case QMessageBox::StandardButton::Cancel:
        leftMenu->getPathGroupDisplayed()->getPathButtonGroup()->uncheck();
        leftMenu->getPathGroupDisplayed()->setLastCheckedButton("");
    break;
    default:
        Q_UNREACHABLE();
        qDebug() << "MainWindow::deletePath you should not be here, you probably forgot to implement the behavior for one of your buttons";
    break;
    }
}

void MainWindow::editPathSlot(QString groupName, QString pathName){
    qDebug() << "MainWindow::editPathSlot called on group :" << groupName << ", path :" << pathName;
    setMessageTop(TEXT_COLOR_INFO, "Click white points of the map to add new points to the path of "
                 "\nAlternatively you can click the \"+\" button to add an existing point to your path"
                 "\nYou can re-order the points in the list by dragging them");

    hideAllWidgets();
    pathCreationWidget->show();
    pathPainter->setOldPath(pathPainter->getCurrentPath());

    switchFocus(pathName, pathCreationWidget, MainWindow::WidgetType::PATH);

    setEnableAll(false, GraphicItemState::CREATING_PATH, true);

    /// stop displaying the currently displayed path if it exists
    emit resetPathCreationWidget();

    pathCreationWidget->setCurrentPathName(pathName);
    if(groupName.compare(""))
        pathCreationWidget->setCurrentGroupName(groupName);
    else
        pathCreationWidget->setCurrentGroupName(pathCreationWidget->getCurrentGroupName());

    bool foundFlag(false);

    leftMenu->getPathGroupDisplayed()->setLastCheckedButton(pathName);
    pathPainter->setVisiblePath(pathName);

    qDebug() << "will edit" <<  groupName << pathName;
    pathCreationWidget->updatePath(paths->getPath(groupName, pathName, foundFlag));

    /// hides the temporary pointview
    points->getTmpPointView()->hide();

    /// displays the points in order to make them available for the edition of the path,
    ///  we have to keep track of those which were hidden so we can hide them again once the edition is finished
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();
        if(i.value() && i.key().compare(PATH_GROUP_NAME) != 0){
            for(int j = 0; j < i.value()->count(); j++){
                if(!i.value()->at(j)->isVisible() && i.value()->at(j)->getPoint()->getName().compare(TMP_POINT_NAME)){
                    i.value()->at(j)->show();
                    pointViewsToDisplay.push_back(i.value()->at(j));
                }
            }
        }
    }
}

void MainWindow::displayPathSlot(QString groupName, QString pathName, bool display){
    qDebug() << "MainWindow::displayPathSlot called on group :" << groupName << ", path :" << pathName << ", display :" << display;
    if(display){
        bool foundFlag = false;
        Paths::Path path = paths->getPath(groupName, pathName, foundFlag);
        if(foundFlag) {
            pathPainter->setCurrentPath(path, pathName);
        }
        else
            qDebug() << "MainWindow::displayPathSlot Sorry could not find the path";
    }
    else {
        emit resetPath();
    }
}

void MainWindow::displayGroupPaths(){
    qDebug() << "MainWindow::displayGroupPaths called";
    switchFocus(leftMenu->getGroupsPathsWidget()->getButtonGroup()->getButtonGroup()->checkedButton()->text(),
                leftMenu->getPathGroupDisplayed(), MainWindow::WidgetType::GROUP_OF_PATHS);

    /// updates the currentGroup displayed
    leftMenu->getpathCreationWidget()->setCurrentGroupName(leftMenu->getGroupsPathsWidget()->getButtonGroup()->getButtonGroup()->checkedButton()->text());

    /// updates the label to display the name of the group
    leftMenu->getPathGroupDisplayed()->getGroupNameLabel()->setText(leftMenu->getGroupsPathsWidget()->getButtonGroup()->getButtonGroup()->checkedButton()->text());
    leftMenu->getPathGroupDisplayed()->setPathsGroup(
                leftMenu->getGroupsPathsWidget()->getButtonGroup()->getButtonGroup()->checkedButton()->text());
    leftMenu->getGroupsPathsWidget()->hide();
    leftMenu->getPathGroupDisplayed()->show();
}

void MainWindow::editGroupPaths(){
    qDebug() << "MainWindow::editGroupPaths called";

    /// resets the line edit
    leftMenu->getGroupsPathsWidget()->getButtonGroup()->getModifyEdit()->setText("");

    leftMenu->getGroupsPathsWidget()->getActionButtons()->getEditButton()->setToolTip("Choose a new name for your group and press the ENTER key");

    int btnIndex = leftMenu->getGroupsPathsWidget()->getButtonGroup()->getButtonGroup()->checkedId();

    leftMenu->getGroupsPathsWidget()->setCreatingGroup(false);

    /// hides the button so we can show the QLineEdit on top of it
    leftMenu->getGroupsPathsWidget()->getButtonGroup()->getButtonGroup()->button(btnIndex)->hide();

    /// disables the buttons
    leftMenu->getGroupsPathsWidget()->getActionButtons()->enableAll(false);

    /// disables the QButtonGroup
    leftMenu->getGroupsPathsWidget()->getButtonGroup()->setEnabledGroup(false);

    leftMenu->getGroupsPathsWidget()->getButtonGroup()->getModifyEdit()->setPlaceholderText(leftMenu->getGroupsPathsWidget()->getLastCheckedButton());
    leftMenu->getGroupsPathsWidget()->getButtonGroup()->getModifyEdit()->setFocus();
    leftMenu->getGroupsPathsWidget()->getButtonGroup()->getModifyEdit()->show();
    leftMenu->getGroupsPathsWidget()->getButtonGroup()->getModifyEdit()->setFocusPolicy(Qt::FocusPolicy::StrongFocus);

    leftMenu->getGroupsPathsWidget()->getButtonGroup()->getLayout()->removeWidget(leftMenu->getGroupsPathsWidget()->getButtonGroup()->getModifyEdit());
    leftMenu->getGroupsPathsWidget()->getButtonGroup()->getLayout()->insertWidget(btnIndex, leftMenu->getGroupsPathsWidget()->getButtonGroup()->getModifyEdit());
}

void MainWindow::createGroupPaths(){
    qDebug() << "MainWindow::createGroupPaths called";
    setMessageTop(TEXT_COLOR_INFO, "The name of your group cannot be empty");

    leftMenu->getGroupsPathsWidget()->setCreatingGroup(true);
    /// unchecks the potential checked button
    leftMenu->getGroupsPathsWidget()->getButtonGroup()->uncheck();

    /// resets the name edit field
    leftMenu->getGroupsPathsWidget()->getGroupNameEdit()->setText("");
    /// stops a user from creating a new group with no name
    leftMenu->getGroupsPathsWidget()->getSaveButton()->setEnabled(false);

    /// uncheck and disable the buttons
    leftMenu->getGroupsPathsWidget()->getActionButtons()->checkAll(false);
    leftMenu->getGroupsPathsWidget()->getActionButtons()->enableAll(false);

    leftMenu->getGroupsPathsWidget()->getActionButtons()->getPlusButton()->setToolTip("Enter a name for your group and click \"save\" or click \"cancel\" to cancel");

    /// here we allow a user to create a new group
    leftMenu->getGroupsPathsWidget()->getGroupNameEdit()->show();
    leftMenu->getGroupsPathsWidget()->getGroupNameLabel()->show();
    leftMenu->getGroupsPathsWidget()->getCancelButton()->show();
    leftMenu->getGroupsPathsWidget()->getSaveButton()->show();

    leftMenu->getGroupsPathsWidget()->getGroupNameEdit()->setFocus();
}

void MainWindow::deleteGroupPaths(){
    qDebug() << "MainWindow::deleteGroupPaths called";
    QString groupPaths(leftMenu->getGroupsPathsWidget()->getButtonGroup()->getButtonGroup()->checkedButton()->text());
    QString message("This group of paths contains the following paths which are assigned to one or more or your robots : ");
    QList<int> robotsIds;
    /// to form a proper sentence
    bool first(true);
    /// we check for each robot whether or not one path of the group has been assigned to it
    for(int i = 0; i < robots->getRobotsVector().size(); i++){
        QString currentRobotPath = robots->getRobotsVector().at(i)->getRobot()->getPathName();
        QMapIterator<QString, QSharedPointer<Paths::Path> > it_paths(paths->getGroup(groupPaths));
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
    message += ". If you delete this group, these robots will lose their paths. If you wish to continue click \"Ok\". ";
    int answer;
    /// if none of the paths of this group has been assigned to a robot
    if(first)
        answer = openConfirmMessage("Are you sure you want to delete this group of path, all the paths inside will be deleted as well ?");
    else
        answer = openConfirmMessage(message);
    leftMenu->getGroupsPathsWidget()->resetWidget();
    switch(answer){
    case QMessageBox::StandardButton::Ok:
    {
        /// to delete the paths of the robots whose paths are contained in the group which is about to be deleted
        while(!robotsIds.empty()){
            clearPath(robotsIds.front());
            robotsIds.pop_front();
        }
        paths->deleteGroup(groupPaths);
        serializePaths(QDir::currentPath() + QDir::separator() + "paths.dat");

        leftMenu->getGroupsPathsWidget()->updateGroupsPaths();
        break;
    }
    case QMessageBox::StandardButton::Cancel:
        /// unchecks the group button (prettier imo but not necessary on the logical level)
        leftMenu->getGroupsPathsWidget()->uncheck();
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
    if(leftMenu->getGroupsPathsWidget()->checkGroupName(name) == 0){
        leftMenu->getGroupsPathsWidget()->setLastCheckedButton("");

        /// updates the model
        paths->createGroup(name);

        /// updates list of groups in menu
        leftMenu->getGroupsPathsWidget()->updateGroupsPaths();

        /// enables the return button again
        leftMenu->getReturnButton()->setEnabled(true);

        /// hides everything that's related to the creation of a group
        leftMenu->getGroupsPathsWidget()->getSaveButton()->hide();
        leftMenu->getGroupsPathsWidget()->getCancelButton()->hide();
        leftMenu->getGroupsPathsWidget()->getGroupNameEdit()->hide();
        leftMenu->getGroupsPathsWidget()->getGroupNameLabel()->hide();

        /// enables the plus button again
        leftMenu->getGroupsPathsWidget()->disableButtons();
        leftMenu->getGroupsPathsWidget()->getActionButtons()->getPlusButton()->setEnabled(true);
        leftMenu->getGroupsPathsWidget()->getActionButtons()->getPlusButton()->setToolTip("Click here to add a new group of paths");
        topLayout->setEnabled(true);
        leftMenu->getGroupsPathsWidget()->getActionButtons()->getPlusButton()->setEnabled(true);  
        serializePaths(QDir::currentPath() + QDir::separator() + "paths.dat");

        topLayout->setLabelDelay(TEXT_COLOR_SUCCESS, "You have created a new group of paths", 4000);
    } else if(leftMenu->getGroupsPathsWidget()->checkGroupName(name) == 1){
        /// enables the return button again
        leftMenu->getReturnButton()->setEnabled(true);

        /// hides everything that's related to the creation of a group
        leftMenu->getGroupsPathsWidget()->getSaveButton()->hide();
        leftMenu->getGroupsPathsWidget()->getCancelButton()->hide();
        leftMenu->getGroupsPathsWidget()->getGroupNameEdit()->hide();
        leftMenu->getGroupsPathsWidget()->getGroupNameLabel()->hide();

        /// enables the plus button again
        leftMenu->getGroupsPathsWidget()->disableButtons();
        leftMenu->getGroupsPathsWidget()->getActionButtons()->getPlusButton()->setEnabled(true);
        leftMenu->getGroupsPathsWidget()->getActionButtons()->getPlusButton()->setToolTip("Click here to add a new group of paths");
        topLayout->setEnabled(true);
        leftMenu->getGroupsPathsWidget()->getActionButtons()->getPlusButton()->setEnabled(true);
    }
    else
        topLayout->setLabelDelay(TEXT_COLOR_DANGER, "You cannot choose : " + name + " as a new name for your group because another group already has this name", 4000);
}

void MainWindow::modifyGroupPathsWithEnter(QString name){
    name = name.simplified();
    qDebug() << "modifying group paths after enter key pressed from" << pointsLeftWidget->getLastCheckedId() << "to" << name;

    leftMenu->setEnableReturnCloseButtons(true);

    /// enables the plus button
    leftMenu->getGroupsPathsWidget()->getActionButtons()->getPlusButton()->setEnabled(true);

    leftMenu->getGroupsPathsWidget()->getButtonGroup()->getModifyEdit()->hide();

    /// if the name has really changed
    if(name.compare(leftMenu->getGroupsPathsWidget()->getLastCheckedButton())){
        /// Update the model
        QSharedPointer<Paths::CollectionPaths> value = paths->getGroups()->value(leftMenu->getGroupsPathsWidget()->getButtonGroup()->getButtonGroup()->checkedButton()->text());
        paths->getGroups()->remove(leftMenu->getGroupsPathsWidget()->getButtonGroup()->getButtonGroup()->checkedButton()->text());
        paths->getGroups()->insert(name, value);
        leftMenu->getGroupsPathsWidget()->updateGroupsPaths();
        topLayout->setLabelDelay(TEXT_COLOR_SUCCESS, "You have successfully modified the name of your group", 4000);
    } else {
        leftMenu->getGroupsPathsWidget()->updateGroupsPaths();
        setMessageTop(TEXT_COLOR_NORMAL, "");
    }

    leftMenu->getGroupsPathsWidget()->setLastCheckedButton("");
}

void MainWindow::doubleClickOnPathsGroup(QString checkedButton){
    leftMenu->getPathGroupDisplayed()->setPathsGroup(checkedButton);
    switchFocus(checkedButton, leftMenu->getPathGroupDisplayed(), MainWindow::WidgetType::GROUP_OF_PATHS);
    leftMenu->getpathCreationWidget()->setCurrentGroupName(checkedButton);
    leftMenu->getPathGroupDisplayed()->getGroupNameLabel()->setText(checkedButton);
    leftMenu->getGroupsPathsWidget()->hide();
    leftMenu->getPathGroupDisplayed()->show();
}

void MainWindow::displayPath(){

    QString groupName = lastWidgets.at(lastWidgets.size()-1).first.second;
    QString pathName = leftMenu->getPathGroupDisplayed()->getLastCheckedButton();
    qDebug() << "MainWindow::displayPath called" << groupName << pathName;
    bool foundFlag = false;
    leftMenu->getDisplaySelectedPath()->updatePath(groupName,
                                                   pathName,
                                                   paths->getPath(groupName, pathName, foundFlag),
                                                   pathPainter->getVisiblePath());
    switchFocus(leftMenu->getPathGroupDisplayed()->getLastCheckedButton(), leftMenu->getDisplaySelectedPath(), MainWindow::WidgetType::PATH);
    leftMenu->getPathGroupDisplayed()->hide();
    leftMenu->getDisplaySelectedPath()->show();
    leftMenu->getPathGroupDisplayed()->getPathButtonGroup()->uncheck();
    leftMenu->getPathGroupDisplayed()->getPathButtonGroup()->setCheckable(true);
}

void MainWindow::createPath(){
    qDebug() << "MainWindow::createPath called";
    /// to prevent a path to be saved with an empty name
    leftMenu->getpathCreationWidget()->getNameEdit()->setText("");
    leftMenu->getpathCreationWidget()->getSaveButton()->setEnabled(false);

    switchFocus("newPath", pathCreationWidget, MainWindow::WidgetType::PATH);

    /// to clear the map of any path
    emit resetPath();

    setMessageTop(TEXT_COLOR_INFO, "The name of your path cannot be empty, fill up the corresponding field to give your path a name");
    hideAllWidgets();
    setEnableAll(false, GraphicItemState::CREATING_PATH, true);
    leftMenu->getpathCreationWidget()->show();

    /// stop displaying the currently displayed path if it exists
    pathCreationWidget->getPathPointList()->clear();
    bottomLayout->uncheckAll();

    /// hides the temporary pointview
    points->getTmpPointView()->hide();

    /// displays the points in order to make them available for the edition of the path,
    ///  we have to keep track of those which were hidden so we can hide them again once the edition is finished
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();
        if(i.value() && i.key().compare(PATH_GROUP_NAME) != 0){
            for(int j = 0; j < i.value()->count(); j++){
                if(!i.value()->at(j)->isVisible() && i.value()->at(j)->getPoint()->getName().compare(TMP_POINT_NAME)){
                    i.value()->at(j)->show();
                    pointViewsToDisplay.push_back(i.value()->at(j));
                }
            }
        }
    }
}

void MainWindow::deletePath(){
    qDebug() << "MainWindow::deletePath called";
    QString message("This path has been assigned to the robot(s) : ");
    /// the ids of the robots whose path is the one being deleted
    QList<int> robotsIds;
    /// to write a well formed sentence
    bool first(true);
    /// we check if this path has been assigned to a robot
    for(int i = 0; i < robots->getRobotsVector().size(); i++){
        if(!robots->getRobotsVector().at(i)->getRobot()->getPathName().compare(leftMenu->getPathGroupDisplayed()->getLastCheckedButton())){
            qDebug() << "The path" << leftMenu->getPathGroupDisplayed()->getLastCheckedButton() <<
                        "has been assigned to the robot" << robots->getRobotsVector().at(i)->getRobot()->getName();
            robotsIds.push_back(i);
            if(first){
                message += robots->getRobotsVector().at(i)->getRobot()->getName();
                first = false;
            } else
                message += ", " + robots->getRobotsVector().at(i)->getRobot()->getName();
        }
    }
    message += ". If you delete this path, these robots will lose their path. Click \"ok\" if you wish to continue.";
    int answer;
    if(first)
        answer  = openConfirmMessage("Are you sure you want to delete this path, this action is irreversible ?");
    else
        answer = openConfirmMessage(message);

    switch(answer){
    case QMessageBox::StandardButton::Ok:
    {
        leftMenu->getPathGroupDisplayed()->initializeActionButtons();
        /// resets the path painter if the path displayed is the one we just destroyed
        qDebug() << "deleting path" << leftMenu->getPathGroupDisplayed()->getLastCheckedButton();

        if(!pathPainter->getVisiblePath().compare(leftMenu->getPathGroupDisplayed()->getLastCheckedButton())){
            qDebug() << "hey i have to stop displaying this path that was destroyed";
            emit resetPath();
        } else
            qDebug() << "dont have to stop displaying this path" << pathPainter->getVisiblePath() << "because the last one checked is" << leftMenu->getPathGroupDisplayed()->getLastCheckedButton();

        /// remove the paths of the robots
        while(!robotsIds.empty()){
            qDebug() << "id : " << robotsIds.front();
            clearPath(robotsIds.front());
            robotsIds.pop_front();
        }

        paths->deletePath(lastWidgets.at(lastWidgets.size()-1).first.second, leftMenu->getPathGroupDisplayed()->getLastCheckedButton());
        serializePaths(QDir::currentPath() + QDir::separator() + "paths.dat");

        leftMenu->getPathGroupDisplayed()->setPathsGroup(lastWidgets.at(lastWidgets.size()-1).first.second);
    }
    break;
    case QMessageBox::StandardButton::Cancel:
        leftMenu->getPathGroupDisplayed()->getPathButtonGroup()->uncheck();
        leftMenu->getPathGroupDisplayed()->setLastCheckedButton("");
    break;
    default:
        Q_UNREACHABLE();
        qDebug() << "MainWindow::deletePath you should not be here, you probably forgot to implement the behavior for one of your buttons";
    break;
    }
}

void MainWindow::displayPathOnMap(const bool display){
    qDebug() << "MainWindow::displayPathOnMap called";
    /// to hide the path drawn by the robot path painter
    emit resetPath();
    if(display){
        bottomLayout->uncheckViewPathSelectedRobot(bottomLayout->getLastCheckedId());
        bool foundFlag = false;
        pathPainter->setCurrentPath(paths->getPath(lastWidgets.at(lastWidgets.size()-1).first.second, leftMenu->getPathGroupDisplayed()->getLastCheckedButton(), foundFlag), leftMenu->getPathGroupDisplayed()->getLastCheckedButton());
    } else {
        pathPainter->setVisiblePath("");
        qDebug() << "no path visible !";
    }
    /// to set the 'eye' icon appropriately
    leftMenu->getPathGroupDisplayed()->updateDisplayedPath();
}

void MainWindow::editPath(){
    qDebug() << "MainWindow::editPath called";
    setMessageTop(TEXT_COLOR_INFO, "Click white points of the map to add new points to the path of "
                 "\nAlternatively you can click the \"+\" button to add an existing point to your path"
                 "\nYou can re-order the points in the list by dragging them");

    hideAllWidgets();
    pathCreationWidget->show();
    pathPainter->setOldPath(pathPainter->getCurrentPath());

    const QString pathName = leftMenu->getPathGroupDisplayed()->getPathButtonGroup()->getButtonGroup()->checkedButton()->text();
    const QString groupName = leftMenu->getPathGroupDisplayed()->getGroupNameLabel()->text();

    switchFocus(pathName, pathCreationWidget, MainWindow::WidgetType::PATH);

    /// stop displaying the currently displayed path if it exists
    emit resetPathCreationWidget();

    bottomLayout->uncheckAll();

    setEnableAll(false, GraphicItemState::CREATING_PATH, true);

    /// to be able to know if the path name is different from the old one
    qDebug() << "setting the current path name to" << pathName;
    pathCreationWidget->setCurrentPathName(pathName);
    pathCreationWidget->setCurrentGroupName(groupName);

    leftMenu->getPathGroupDisplayed()->setLastCheckedButton(pathName);
    pathPainter->setVisiblePath(pathName);

    qDebug() << "will edit" <<  groupName << pathName;
    bool foundFlag(false);
    pathCreationWidget->updatePath(paths->getPath(leftMenu->getPathGroupDisplayed()->getGroupNameLabel()->text(),
                                                         leftMenu->getPathGroupDisplayed()->getPathButtonGroup()->getButtonGroup()->checkedButton()->text(), foundFlag));

    /// hides the temporary pointview
    points->getTmpPointView()->hide();

    /// displays the points in order to make them available for the edition of the path,
    ///  we have to keep track of those which were hidden so we can hide them again once the edition is finished
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();
        if(i.value() && i.key().compare(PATH_GROUP_NAME) != 0){
            for(int j = 0; j < i.value()->count(); j++){
                if(!i.value()->at(j)->isVisible() && i.value()->at(j)->getPoint()->getName().compare(TMP_POINT_NAME)){
                    i.value()->at(j)->show();
                    pointViewsToDisplay.push_back(i.value()->at(j));
                }
            }
        }
    }
}

void MainWindow::doubleClickOnPath(QString pathName){
    QString groupName = lastWidgets.at(lastWidgets.size()-1).first.second;
    qDebug() << "MainWindow::displayPath called" << groupName << pathName;
    bool foundFlag = false;
    leftMenu->getDisplaySelectedPath()->updatePath(groupName,
                                                   pathName,
                                                   paths->getPath(groupName, pathName, foundFlag),
                                                   pathPainter->getVisiblePath());
    switchFocus(pathName, leftMenu->getDisplaySelectedPath(), MainWindow::WidgetType::PATH);
    leftMenu->getPathGroupDisplayed()->hide();
    leftMenu->getDisplaySelectedPath()->show();
    leftMenu->getPathGroupDisplayed()->getPathButtonGroup()->uncheck();
}

void MainWindow::sendPathSelectedRobotSlot(const QString groupName, const QString pathName){

    qDebug() << "MainWindow::sendPathSelectedRobotSlot path changed";
    QPointer<Robot> robot = selectedRobot->getRobot();
    /// prepares the cmd to send to the robot
    qDebug() << "MainWindow::sendPathSelectedRobotSlot" << pathPainter->getCurrentPath().size();
    bool flag;
    Paths::Path currPath = paths->getPath(groupName, pathName, flag);
    QString pathStr = prepareCommandPath(currPath);

    /// if the command is succesfully sent to the robot, we apply the change
    // TODO check if pathStr empty, means deleting the path => cmd send without parem => check robotFiles/command.cpp
    if(commandController->sendCommand(robot, QString("i ") + pathStr)){
        /// we update the path on the application side by serializing the path
        QFile fileInfo(QDir::currentPath() + QDir::separator() + "robots_paths" + QDir::separator() + robot->getName() + "_path");
        if(fileInfo.open(QIODevice::ReadWrite)){
            fileInfo.resize(0);
            QTextStream out(&fileInfo);
            QString currentDateTime = QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss");
            out << currentDateTime;
            out << "%" << groupName << "%" << pathName;
            qDebug() << "date now is" << currentDateTime;
            fileInfo.close();
            editSelectedRobotWidget->setPath(currPath);
        }
        qDebug() << "MainWindow::robotSavedEvent Path saved for robot" << robot->getIp();
    } else {
        qDebug() << "MainWindow::robotSavedEvent Path failed to be saved, please try again";
    }
}

void MainWindow::setMessageNoRobotPath(const int code){
    switch(code){
    case 0:
        setMessageTop(TEXT_COLOR_INFO, "You cannot save your path because its name is still empty");
        leftMenu->getpathCreationWidget()->getSaveButton()->setEnabled(false);
    break;
    case 1:
        setMessageTop(TEXT_COLOR_INFO, "You cannot save your path because the name you chose is already taken by another path in the same group");
        leftMenu->getpathCreationWidget()->getSaveButton()->setEnabled(false);
    break;
    case 2:
        setMessageTop(TEXT_COLOR_INFO, "You can save your path any time you want by clicking the \"Save\" button");
        leftMenu->getpathCreationWidget()->getSaveButton()->setEnabled(true);
    break;
    default:
        Q_UNREACHABLE();
        qDebug() << "MainWindow::setMessageNoRobotPath you should not be here you probably forgot to implement the behavior for the error code" << code;
    break;
    }
}

void MainWindow::cancelNoRobotPathSlot(){
    qDebug() << "MainWindow::cancelNoRobotPathSlot called";

    leftMenu->getPathGroupDisplayed()->setPathsGroup(pathCreationWidget->getCurrentGroupName());

    QVector<QSharedPointer<PathPoint>> oldPath = pathPainter->getOldPath();
    /// we hide the points that we displayed just for the edition of the path
    for(int i = 0; i < pointViewsToDisplay.size(); i++)
        pointViewsToDisplay.at(i)->hide();
    pointViewsToDisplay.clear();

    emit resetPathCreationWidget();

    pathCreationWidget->updatePath(oldPath);

    backEvent();

    pathPainter->setOldPath(oldPath);
    setEnableAll(false, GraphicItemState::NO_EVENT);
    leftMenu->setEnableReturnCloseButtons(true);
    topLayout->setEnable(true);
    bottomLayout->setEnable(true);

    setTemporaryMessageTop(TEXT_COLOR_INFO, "You have cancelled the modifications of the path \"" + pathCreationWidget->getCurrentPathName() + "\"", 2500);
}

void MainWindow::saveNoRobotPathSlot(){
    qDebug() << "MainWindow::saveNoRobotPath called";
    backEvent();

    /// gotta update the model and serialize the paths
    const QString groupName = pathCreationWidget->getCurrentGroupName();
    const QString pathName = pathCreationWidget->getNameEdit()->text().simplified();
    qDebug() << groupName << pathName;
    paths->createPath(groupName, pathName);
    for(int i = 0; i < pathPainter->getCurrentPath().size(); i++)
        paths->addPathPoint(groupName, pathName, pathPainter->getCurrentPath().at(i));

    /// we hide the points that we displayed for the edition of the path
    for(int i = 0; i < pointViewsToDisplay.size(); i++){
        bool hidePointView(true);
        for(int j = 0; j < pathPainter->getCurrentPath().size(); j++){
            if(pathPainter->getCurrentPath().at(j)->getPoint() == *(pointViewsToDisplay.at(i)->getPoint())){
                hidePointView = false;
                break;
            }
        }
        if(hidePointView)
            pointViewsToDisplay.at(i)->hide();
    }
    pointViewsToDisplay.clear();

    pathPainter->setPathDeleted(false);
    pathPainter->setOldPath(pathPainter->getCurrentPath());

    setEnableAll(false, GraphicItemState::NO_EVENT);

    leftMenu->setEnableReturnCloseButtons(true);

    /// updates the visible path
    pathPainter->setVisiblePath(pathName);

    /// resets the menu so that it reflects the creation of this new path
    leftMenu->getPathGroupDisplayed()->setPathsGroup(pathCreationWidget->getCurrentGroupName());

    /// add this path to the file
    serializePaths(QDir::currentPath() + QDir::separator() + "paths.dat");

    //editSelectedRobotWidget->setPathChanged(true);
    //editSelectedRobotWidget->setPath(pathPainter->getCurrentPath());
    //emit updatePathPainter();
}

void MainWindow::setMessageModifGroupPaths(int code){
    switch(code){
    case 0:
        setMessageTop(TEXT_COLOR_INFO, "Press enter to save this name for your group");
        break;
    case 1:
        setMessageTop(TEXT_COLOR_INFO, "You cannot have an empty name for your group");
        break;

    case 2:
        setMessageTop(TEXT_COLOR_INFO, "You cannot save this name for your group as it is already the name of another group");
        break;
    default:
        Q_UNREACHABLE();
        qDebug() << "MainWindow::setMessageModifGroupPaths You should not be here you probably forgot to implement the behavior for the code" << code;
    }
}

void MainWindow::displayAssignedPath(QString groupName, QString pathName){

    bool foundFlag(true);
    pathPainter->setCurrentPath(paths->getPath(groupName, pathName, foundFlag), pathName);
    editSelectedRobotWidget->updatePathsMenu();
    assert(foundFlag);

    selectedRobot->getRobot()->setPath(pathPainter->getCurrentPath());
    selectedRobot->getRobot()->setGroupPathName(editSelectedRobotWidget->getGroupPathName());
    selectedRobot->getRobot()->setPathName(editSelectedRobotWidget->getPathName());
    int id = robots->getRobotId(selectedRobot->getRobot()->getName());
    bottomLayout->updateRobot(id, selectedRobot);
    if(pathPainter->getCurrentPath().size() > 0){
        bottomLayout->getViewPathRobotBtnGroup()->button(id)->setChecked(true);
        viewPathSelectedRobot(id, true);
    }

    /// we update the path on the application side by serializing the path
    QFile fileInfo(QDir::currentPath() + QDir::separator() + "robots_paths" + QDir::separator() + selectedRobot->getRobot()->getName() + "_path");
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

void MainWindow::updateModelPaths(const Point& old_point, const Point& new_point){
    qDebug() << "name of the point which caused the paths to be updated" << old_point.getName() << "new name" << new_point.getName();
    paths->updatePaths(old_point, new_point);
    /// saves the paths as the paths of the current configuration
    /// for paths to be saved permanently "save map" must be clicked (map menu)
    serializePaths(QDir::currentPath() + QDir::separator() + "paths.dat");
}

/**********************************************************************************************************************************/

//                                          ODDS AND ENDS

/**********************************************************************************************************************************/

void MainWindow::quit(){
    close();
}

void MainWindow::backEvent(){
    //qDebug() << "back event called";

    setEnableAll(true);
    /// resets the menus
    pointsLeftWidget->disableButtons();
    pointsLeftWidget->getGroupButtonGroup()->uncheck();
    leftMenu->getDisplaySelectedGroup()->disableButtons();
    leftMenu->getDisplaySelectedGroup()->uncheck();

    if(lastWidgets.size() > 0)
        lastWidgets.last().first.first->hide();

    if (lastWidgets.size() > 1){
        lastWidgets.removeLast();
        if(lastWidgets.last().second == MainWindow::WidgetType::GROUP || lastWidgets.last().second == MainWindow::WidgetType::GROUPS)
            setMessageTop(TEXT_COLOR_INFO, "Click the map to add a permanent point");

        lastWidgets.last().first.first->show();

        if (lastWidgets.size() >1){
            leftMenu->showBackButton(lastWidgets.at(lastWidgets.size()-2).first.second);
        } else {
            leftMenu->hideBackButton();
        }
    } else {
        resetFocus();
        leftMenu->hide();
    }

    //debug
    qDebug() << "_________________";

    for(int i=0;i<lastWidgets.size();i++) {
        qDebug() << lastWidgets.at(i).first.second;
    }
    qDebug() << "_________________";

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
    qDebug() << "MainWindow::setGraphicItemsState called" << state << robots->getRobotsVector().size();
    mapPixmapItem->setState(state);

    for(int i = 0; i < robots->getRobotsVector().size(); i++){
        robots->getRobotsVector().at(i)->setState(state);
    }

    points->setPointViewsState(state);
}

void MainWindow::hideAllWidgets(){
    leftMenuWidget->hide();
    pointsLeftWidget->hide();
    robotsLeftWidget->hide();
    mapLeftWidget->hide();
    editSelectedRobotWidget->hide();
    createPointWidget->hide();
    leftMenu->getDisplaySelectedPoint()->hide();
    pathCreationWidget->hide();
    leftMenu->getDisplaySelectedGroup()->hide();
    leftMenu->getGroupsPathsWidget()->hide();
    leftMenu->getDisplaySelectedPath()->hide();
    leftMenu->getPathGroupDisplayed()->hide();
}

void MainWindow::clearNewMap(){
    qDebug() << "clearNewMap called";

    selectedPoint = QSharedPointer<PointView>();
    editedPointView = QSharedPointer<PointView>();

    pathPainter->setVisiblePath("");
    emit resetPath();

    /// Clear the list of points
    points->clear();

    /// clears the list of paths
    paths->clear();

    /// Update the left menu displaying the list of groups and buttons
    pointsLeftWidget->updateGroupButtonGroup();

    for(int i = 0; i < robots->getRobotsVector().size(); i++)
        robots->getRobotsVector().at(i)->getRobot()->clearPath();
}

void MainWindow::delay(const int ms){
    QTime dieTime= QTime::currentTime().addMSecs(ms);
    while (QTime::currentTime() < dieTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

void MainWindow::setEnableAll(bool enable, GraphicItemState state, int noReturn){
    setGraphicItemsState(state);
    bottomLayout->setEnable(enable);
    topLayout->setEnable(enable);
    if(noReturn == -1)
        leftMenu->setEnableReturnCloseButtons(enable);
    else
        leftMenu->setEnableReturnCloseButtons(noReturn);
}

void MainWindow::centerMap(){
    /// centers the map on the point chosen by the user and not on the center of the map
    mapPixmapItem->setPos(mapState.first);
    graphicsView->centerOn(mapState.first);
    if(graphicsView->getZoomCoeff() != mapState.second)
        graphicsView->setZoomCoeff(mapState.second);
}

void MainWindow::settingBtnSlot(){
    qDebug() << "MainWindow::settingBtnSlot called";
    //editMapSlot();

    if(robots->getRobotsVector().size() > 0)
        commandController->sendCommand(robots->getRobotsVector().at(0)->getRobot(), QString("s"));
}

void MainWindow::setTemporaryMessageTop(const QString type, const QString message, const int ms){
    setMessageTop(type, message);
    delay(ms);
    setMessageTop(TEXT_COLOR_NORMAL, "");
}

void MainWindow::saveMapState(){
    qDebug() << "MainWindow::saveMapState saving map" << QString::fromStdString(mapFile);
    /// saves the current configuration in the model
    //mapState.first = mapPixmapItem->pos();
    //mapState.second = graphicsView->getZoomCoeff();

    QFileInfo newMapInfo(QDir::currentPath(), "../gobot-software/currentMap.txt");
    std::ofstream file(newMapInfo.absoluteFilePath().toStdString(), std::ios::out | std::ios::trunc);

    /// saves the current configuration into the current configuration file
    if(file){
        file << mapFile << " " << std::endl <<
                map->getHeight() << " " << map->getWidth() << std::endl
             << mapState.first.x() << " " << mapState.first.y() << std::endl
             << mapState.second << std::endl
             << map->getOrigin().getX() << " " << map->getOrigin().getY() << std::endl << map->getResolution() << std::endl
             << map->getDateTime().toString("yyyy-MM-dd-hh-mm-ss").toStdString() << std::endl
             << map->getMapId().toString().toStdString();
        file.close();
    }
}

void MainWindow::showHomes(){
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();
        if(i.value() && i.key().compare(PATH_GROUP_NAME) != 0 && i.key().compare(TMP_GROUP_NAME)){
            for(int j = 0; j < i.value()->count(); j++){
                if(i.value()->at(j)->getPoint()->isHome()){
                    if(!i.value()->at(j)->getPoint()->getRobotName().compare(selectedRobot->getRobot()->getName())){
                        /// to make the old home a normal point again
                        i.value()->at(j)->getPoint()->setHome(Point::HOME);
                        i.value()->at(j)->setPixmap(PointView::PixmapType::NORMAL);
                    } else {
                        /// to make it look like a normal point we alter its type temporarily
                        i.value()->at(j)->getPoint()->setHome(Point::PERM);
                        i.value()->at(j)->setPixmap(PointView::PixmapType::NORMAL);
                        i.value()->at(j)->getPoint()->setHome(Point::HOME);
                    }
                }
            }
        }
    }
}

void MainWindow::showHomes(QPointer<Robot> robot){
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();
        if(i.value() && i.key().compare(PATH_GROUP_NAME) != 0 && i.key().compare(TMP_GROUP_NAME)){
            for(int j = 0; j < i.value()->count(); j++){
                qDebug() << i.value()->at(j)->getPoint()->getRobotName();
                if(i.value()->at(j)->getPoint()->isHome()){
                    if(i.value()->at(j)->getPoint()->getRobotName().compare(robot->getName())){
                        /// to make it look like a normal point we alter its type temporarily
                        i.value()->at(j)->getPoint()->setHome(Point::PERM);
                        i.value()->at(j)->setPixmap(PointView::PixmapType::NORMAL);
                        i.value()->at(j)->getPoint()->setHome(Point::HOME);
                    }
                }
            }
        }
    }
}

void MainWindow::showSelectedRobotHomeOnly(){
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();
        if(i.value() && i.key().compare(PATH_GROUP_NAME) != 0 && i.key().compare(TMP_GROUP_NAME)){
            for(int j = 0; j < i.value()->count(); j++){
                qDebug() << i.value()->at(j)->getPoint()->getRobotName();
                if(i.value()->at(j)->getPoint()->isHome()){
                    if(i.value()->at(j)->getPoint()->getName().compare(selectedRobot->getRobot()->getHome()->getPoint()->getName())){
                        /// to make it look like a normal point we alter its type temporarily
                        qDebug() << "robot's home name" << selectedRobot->getRobot()->getHome()->getPoint()->getName() << i.value()->at(j)->getPoint()->getName();
                        i.value()->at(j)->getPoint()->setHome(Point::PERM);
                        i.value()->at(j)->setPixmap(PointView::PixmapType::NORMAL);
                        i.value()->at(j)->getPoint()->setHome(Point::HOME);
                    } else
                        qDebug() << "robot's home" << selectedRobot->getRobot()->getName() << i.value()->at(j)->getPoint()->getName();
                }
            }
        }
    }
}

void MainWindow::compress(const QString zipFile){
    ZipWriter cZip(zipFile);
    const QString SingleFile = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + QString("lolilol.pgm");
    QFile file(SingleFile);
    file.open(QIODevice::ReadOnly);
    cZip.addFile("test.pgm", file.readAll());
    file.close();
    cZip.close();
}

void MainWindow::decompress(const QString fileName){
    ZipReader cZip(fileName);
    ZipReader::FileInfo fInfo = cZip.entryInfoAt(0);
    QFile file(QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + QString("lolilol2.pgm"));
    file.open(QIODevice::WriteOnly);
    file.write(cZip.fileData(fInfo.filePath));
    file.close();
    cZip.close();
}

bool MainWindow::saveMapConfig(const std::string fileName){
    qDebug() << "MainWindow::saveMapConfig saving map to " << QString::fromStdString(fileName);
    std::ofstream file(fileName, std::ios::out | std::ios::trunc);
    if(file){
        file << mapFile << " " << std::endl <<
                map->getHeight() << " " << map->getWidth() << std::endl
             << mapState.first.x() << " " << mapState.first.y() << std::endl
             << mapState.second << std::endl
             << map->getOrigin().getX() << " " << map->getOrigin().getY() << std::endl
             << map->getResolution() << std::endl
             << map->getMapId().toString().toStdString();
        file.close();
        return true;
    } else
        return false;
}

bool MainWindow::loadMapConfig(const std::string fileName){
    /// loads the configuration contained in the file <fileName> into the application
    std::ifstream file(fileName, std::ios::in);
    if(file){
        int _height, _width;
        double centerX, centerY, originX, originY, resolution;
        std::string mapId;
        file >> mapFile >> _height >> _width >> centerX >> centerY >> mapState.second >> originX >> originY >> resolution >> mapId;
        map->setHeight(_height);
        map->setWidth(_width);
        mapState.first.setX(centerX);
        mapState.first.setY(centerY);
        map->setOrigin(Position(originX, originY));
        map->setResolution(resolution);
        map->setMapId(QUuid(QString::fromStdString(mapId)));
        setWindowTitle(QString::fromStdString(mapFile));
        file.close();
    } else
        return false;

    /// saves the configuration contained in the file <fileName> as the current configuration
    return saveMapConfig((QDir::currentPath() + QDir::separator() + "currentMap.txt").toStdString());
}

void MainWindow::updateRobotInfo(QString robot_name, QString robotInfo){

    updatePathInfo(robot_name, robotInfo);

    updateHomeInfo(robot_name, robotInfo);

}

bool MainWindow::isLater(const QStringList& date, const QStringList& otherDate){
    for(int i = 0; i < date.size(); i++){
        if(date.at(i).toInt() > otherDate.at(i).toInt())
            return true;
        else if(date.at(i).toInt() < otherDate.at(i).toInt())
            return false;
    }
    return false;
}

QPair<Position, QStringList> MainWindow::getHomeFromFile(const QString robot_name){
    /// retrieves the home point of the robot if the robot has one
    QFile fileInfo(QDir::currentPath() + QDir::separator() + "robots_homes" + QDir::separator() + robot_name);
    Position p;
    QStringList dateLastModification;
    if(fileInfo.open(QIODevice::ReadWrite)){
        QRegExp regex("[-\n ]");
        QString content = fileInfo.readAll();
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

void MainWindow::setHomeAtConnection(const QString robot_name, const Position &pos_home){
    /// the robot and the application have the same point so we set this one as the home of the robot
    QSharedPointer<PointView> home = points->findPointViewByPos(pos_home);
    QPointer<RobotView> robotView = robots->getRobotViewByName(robot_name);
    /// Remove the previous home
    if(robotView->getRobot()->getHome()){
        robotView->getRobot()->getHome()->getPoint()->setHome(Point::PERM);
        robotView->getRobot()->getHome()->setPixmap(PointView::PixmapType::NORMAL);
    }

    /// associates the robot to the point
    home->getPoint()->setRobotName(robotView->getRobot()->getName());
    robotView->getRobot()->setHome(home);

    /// setCurrentPath is displaying the path so if it was not displayed we hide it
    if(!bottomLayout->getViewPathRobotBtnGroup()->button(robots->getRobotId(robotView->getRobot()->getName()))->isChecked())
        emit resetPath();

    savePoints(QDir::currentPath() + QDir::separator() + "points.xml");
}

bool MainWindow::updateHomeFile(const QString robot_name, const Position& robot_home_position, const QStringList date){
    QFile fileWriteHome(QDir::currentPath() + QDir::separator() + "robots_homes" + robot_name);
    if(fileWriteHome.open(QIODevice::ReadWrite)){
        QTextStream out(&fileWriteHome);
        out << robot_home_position.getX() << " " << robot_home_position.getY() << "\n";
        for(int i = 0; i < date.size()-1; i++)
            out << date.at(i) << "-";
        out << date.at(date.size()-1);
        fileWriteHome.close();
        return true;
    } else {
        qDebug() << "could not update the home of" << robot_name;
        return false;
    }
}

QVector<PathPoint> MainWindow::extractPathFromInfo(const QStringList &robotInfo){
    QVector<PathPoint> path;
    for(int i = 5; i < robotInfo.size(); i += 3){
        double xOnRobot = robotInfo.at(i).toDouble();
        double xInApp = (-map->getOrigin().getX() + xOnRobot) / map->getResolution() + ROBOT_WIDTH;
        double yOnRobot = robotInfo.at(i+1).toDouble();
        double yInApp = map->getHeight()-(-map->getOrigin().getY()+yOnRobot) / map->getResolution()-ROBOT_WIDTH/2;
        path.push_back(PathPoint(Point(" ", xInApp, yInApp), robotInfo.at(i+2).toDouble()));
    }
    return path;
}

void MainWindow::updateHomeInfo(const QString robot_name, QString robotInfo){
    QPointer<RobotView> robotView = robots->getRobotViewByName(robot_name);

    /// retrieves the home point of the robot if the robot has one
    QPair<Position, QStringList> appHome = getHomeFromFile(robot_name);
    Position p = appHome.first;
    QStringList dateLastModification = appHome.second;

    robotInfo.replace("\n", " ");

    QStringList list = robotInfo.split(" ", QString::SkipEmptyParts);
    Position robot_home_position(list.at(1).toDouble(), list.at(2).toDouble());
    QStringList dateHomeOnRobot = list.at(3).split("-");

    /// if the robot and the application have the same home we don't do anything besides setting the point in the application (no need to change any files)
    if(robot_home_position != p){
         qDebug() << "HEY HOMES ARE DIFFERENT" << robot_home_position.getX() << robot_home_position.getY()
                  << p.getX() << p.getY();
         QSharedPointer<PointView> home = points->findPointViewByPos(p);

         if(home){

            /// the application has a home, we need to compare with the robot's home if one exists
            QSharedPointer<PointView> home_sent_by_robot = points->findPointViewByPos(robot_home_position);
            if(home_sent_by_robot){
                qDebug() << "HOME ROBOT" << home->getPoint()->getName();
                /// if the robot's file is more recent we update on the application side and we look for the pointview corresponding to
                /// the coordinates given by the robot
                if(isLater(dateHomeOnRobot, dateLastModification)){
                    setHomeAtConnection(robot_name, robot_home_position);
                    updateHomeFile(robot_name, robot_home_position, dateHomeOnRobot);
                } else {
                    /// the application has the most recent file, we send the updated coordinates to the robot
                    if(sendHomeToRobot(robotView, home))
                        setHomeAtConnection(robot_name, p);
                }
            } else {
                qDebug() << "HOME APP" << home->getPoint()->getName();
                if(sendHomeToRobot(robotView, home))
                    setHomeAtConnection(robot_name, p);
            }

        } else {
            qDebug() << " the robot is sending its home and the application does not have one";
            /// if the application does not have a home stored on its side but the robot is sending one
            /// that we are able to find
            QSharedPointer<PointView> home_sent_by_robot = points->findPointViewByPos(robot_home_position);
            if(home_sent_by_robot){
                qDebug() << "HOME ROBOT" << home_sent_by_robot->getPoint()->getName();
                /// sets the home inside the application (house icon, extra buttons etc...)
                setHomeAtConnection(robot_name, robot_home_position);

                /// updates the home file on the application side
                updateHomeFile(robot_name, robot_home_position, dateHomeOnRobot);

            } else
                qDebug() << "HOME ROBOT NO HOME AT ALL";
        }
    } else {
        /// the robot and the application have the same point so we set this one as the home of the robot
        /// no need to modify any files
        QSharedPointer<PointView> home_app = points->findPointViewByPos(p);
        if(home_app){
            setHomeAtConnection(robot_name, p);
            qDebug() << "HOME APP" << home_app->getPoint()->getName();
        } else
            qDebug() << "HOME APP NO HOME AT ALL";
    }
}

void MainWindow::updatePathInfo(const QString robot_name, QString robotInfo){
    QPointer<RobotView> robotView = robots->getRobotViewByName(robot_name);

    /// retrieves the path of the robot on the application side if the robot has one
    QPair<QPair<QString, QString>, QStringList> appPathInfo = getPathFromFile(robot_name);

    robotInfo.replace("\n", " ");

    QStringList l = robotInfo.split(" ", QString::SkipEmptyParts);

    qDebug() << "updatepathinfo list" << l;

    QVector<PathPoint> robotPath = extractPathFromInfo(l);

    /// contains the groupname and pathname of the path described by the robot
    QPair<QString, QString> robotPathInApp = paths->findPath(robotPath);

    /// if the robot has a path
    if(robotPathInApp.first.compare("")){

        bool flag(false);
        paths->getPath(appPathInfo.first.first, appPathInfo.first.second, flag);

        /// the application also has a path for this robot
        if(flag){
            /// they have different paths so we keep the most recent one
            if(robotPathInApp.first.compare(appPathInfo.first.first) || robotPathInApp.second.compare(appPathInfo.first.second)){
                qDebug() << "mainWindow::updatepathinfo DIFFERENT PATHS";
                /// the file is more recent on the robot
                /// we do as if the application did not have a path at all
                if(isLater(l.at(4).split("-", QString::SkipEmptyParts), appPathInfo.second)){
                    qDebug() << " BUT ROBOT MORE RECENT";
                    /// the application does not have a path so we use the path sent by the robot and update the file on the app side
                    bool foundFlag(true);
                    pathPainter->setCurrentPath(paths->getPath(robotPathInApp.first, robotPathInApp.second, foundFlag), robotPathInApp.second);
                    editSelectedRobotWidget->setAssignedPath(robotPathInApp.second);
                    editSelectedRobotWidget->setGroupPath(robotPathInApp.first);
                    pathPainter->setVisiblePath(robotPathInApp.second);
                    robotView->getRobot()->setPath(pathPainter->getCurrentPath());
                    robotView->getRobot()->setGroupPathName(robotPathInApp.first);
                    robotView->getRobot()->setPathName(robotPathInApp.second);
                    int id = robots->getRobotId(robotView->getRobot()->getName());
                    bottomLayout->updateRobot(id, robotView);
                    if(pathPainter->getCurrentPath().size() > 0){
                        bottomLayout->getViewPathRobotBtnGroup()->button(id)->setChecked(true);
                        viewPathSelectedRobot(id, true);
                    }
                    QFile fileInfo(QDir::currentPath() + QDir::separator() + "robots_paths" + QDir::separator()
                                   + robot_name + "_path");
                    if(fileInfo.open(QIODevice::ReadWrite)){
                        fileInfo.resize(0);
                        QTextStream out(&fileInfo);
                        QString currentDateTime = QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss");
                        /// contains the date of the last modification of the path file on the robot
                        out << l.at(4);
                        out << "%" << robotPathInApp.first << "%" << robotPathInApp.second;
                        qDebug() << "date now is" << l.at(4);
                        fileInfo.close();
                    }

                } else {
                    qDebug() << "BUT APP MORE RECENT";
                    /// the file is more recent on the application side
                    /// we sent the path to the robot
                    /// but the application has one
                    bool flag(false);
                    Paths::Path currPath = paths->getPath(appPathInfo.first.first, appPathInfo.first.second, flag);
                    QString pathStr = prepareCommandPath(currPath);
                    if(flag){
                        if(commandController->sendCommand(robotView->getRobot(), QString("i ") + pathStr)){
                            bool foundFlag(false);
                            pathPainter->setCurrentPath(paths->getPath(appPathInfo.first.first, appPathInfo.first.second, foundFlag), appPathInfo.first.second);
                            editSelectedRobotWidget->setAssignedPath(appPathInfo.first.second);
                            pathPainter->setVisiblePath(robotPathInApp.second);
                            editSelectedRobotWidget->setGroupPath(appPathInfo.first.first);
                            robotView->getRobot()->setPath(pathPainter->getCurrentPath());
                            robotView->getRobot()->setGroupPathName(appPathInfo.first.first);
                            robotView->getRobot()->setPathName(appPathInfo.first.second);
                            int id = robots->getRobotId(robot_name);
                            bottomLayout->updateRobot(id, robotView);
                            if(pathPainter->getCurrentPath().size() > 0){
                                bottomLayout->getViewPathRobotBtnGroup()->button(id)->setChecked(true);
                                viewPathSelectedRobot(id, true);
                            }
                        }
                    }
                }
            }
            /// they have the same path
            else {
                qDebug() << "mainWindow::updatepathinfo SAME PATH";
                bool foundFlag(true);
                pathPainter->setCurrentPath(paths->getPath(robotPathInApp.first, robotPathInApp.second, foundFlag), robotPathInApp.second);
                editSelectedRobotWidget->setAssignedPath(robotPathInApp.second);
                pathPainter->setVisiblePath(robotPathInApp.second);
                editSelectedRobotWidget->setGroupPath(robotPathInApp.first);
                robotView->getRobot()->setPath(pathPainter->getCurrentPath());
                robotView->getRobot()->setGroupPathName(robotPathInApp.first);
                robotView->getRobot()->setPathName(robotPathInApp.second);
                int id = robots->getRobotId(robotView->getRobot()->getName());
                bottomLayout->updateRobot(id, robotView);
                if(pathPainter->getCurrentPath().size() > 0){
                    bottomLayout->getViewPathRobotBtnGroup()->button(id)->setChecked(true);
                    viewPathSelectedRobot(id, true);
                }
            }
        } else {
            qDebug() << "mainWindow::updatepathinfo ONLY ROBOT HAS A PATH";
            /// the application does not have a path so we use the path sent by the robot and update the file on the app side
            bool foundFlag(true);
            pathPainter->setCurrentPath(paths->getPath(robotPathInApp.first, robotPathInApp.second, foundFlag), robotPathInApp.second);
            editSelectedRobotWidget->setAssignedPath(robotPathInApp.second);
            pathPainter->setVisiblePath(robotPathInApp.second);
            editSelectedRobotWidget->setGroupPath(robotPathInApp.first);
            robotView->getRobot()->setPath(pathPainter->getCurrentPath());
            robotView->getRobot()->setGroupPathName(robotPathInApp.first);
            robotView->getRobot()->setPathName(robotPathInApp.second);
            int id = robots->getRobotId(robotView->getRobot()->getName());
            bottomLayout->updateRobot(id, robotView);
            if(pathPainter->getCurrentPath().size() > 0){
                bottomLayout->getViewPathRobotBtnGroup()->button(id)->setChecked(true);
                viewPathSelectedRobot(id, true);
            }
            QFile fileInfo(QDir::currentPath() + QDir::separator() + "robots_paths" + QDir::separator()
                           + robot_name + "_path");
            if(fileInfo.open(QIODevice::ReadWrite)){
                fileInfo.resize(0);
                QTextStream out(&fileInfo);
                QString currentDateTime = QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss");
                /// contains the date of the last modification of the path file on the robot
                out << l.at(4);
                out << "%" << robotPathInApp.first << "%" << robotPathInApp.second;
                qDebug() << "date now is" << l.at(4);
                fileInfo.close();
            }
        }
    } else {
        /// the robot does not have a path
        bool flag(false);
        Paths::Path currPath = paths->getPath(appPathInfo.first.first, appPathInfo.first.second, flag);
        if(flag){
            qDebug() << "mainWindow::updatepathinfo ONLY APP HAS A PATH";
            /// but the application has one
            /// prepares the cmd to send to the robot
            QString pathStr = prepareCommandPath(currPath);

            if(commandController->sendCommand(robotView->getRobot(), QString("i ") + pathStr)){
                bool foundFlag(false);
                pathPainter->setCurrentPath(paths->getPath(appPathInfo.first.first, appPathInfo.first.second, foundFlag), appPathInfo.first.second);
                editSelectedRobotWidget->setAssignedPath(appPathInfo.first.second);
                pathPainter->setVisiblePath(robotPathInApp.second);
                editSelectedRobotWidget->setGroupPath(appPathInfo.first.first);
                robotView->getRobot()->setPath(pathPainter->getCurrentPath());
                robotView->getRobot()->setGroupPathName(appPathInfo.first.first);
                robotView->getRobot()->setPathName(appPathInfo.first.second);
                int id = robots->getRobotId(robot_name);
                bottomLayout->updateRobot(id, robotView);
                if(pathPainter->getCurrentPath().size() > 0){
                    bottomLayout->getViewPathRobotBtnGroup()->button(id)->setChecked(true);
                    viewPathSelectedRobot(id, true);
                }
            }
        }
        else qDebug() << "mainWindow::updatepathinfo NO PATH ON EITHER SIDE";
    }
}

QPair<QPair<QString, QString>, QStringList> MainWindow::getPathFromFile(const QString robot_name){
    /// QPair<QPair<groupName, pathName>, date>
    QPair<QPair<QString, QString>, QStringList> pathInfo;
    QFile fileInfo(QDir::currentPath() + QDir::separator() + "robots_paths" + QDir::separator() + robot_name + "_path");
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

        float newPosX = (oldPosX - ROBOT_WIDTH) * map->getResolution() + map->getOrigin().getX();
        float newPosY = (-oldPosY + map->getHeight() - ROBOT_WIDTH/2) * map->getResolution() + map->getOrigin().getY();

        int waitTime = pathPoint->getWaitTime();

        pathStr += + "\"" + QString::number(newPosX) + "\" \"" + QString::number(newPosY) + "\" \"" + QString::number(waitTime)+ "\" ";   
    }
    //qDebug() << "pathstr yo" << pathStr;
    return pathStr;
}
