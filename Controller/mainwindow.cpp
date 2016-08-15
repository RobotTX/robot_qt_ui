#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "Controller/scanmapthread.h"
#include "Controller/updaterobotsthread.h"
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
#include "View/selectedrobotwidget.h"
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
#include "View/customizedlineedit.h"
#include "View/buttonmenu.h"
#include "View/pathpointcreationwidget.h"
#include "View/pathpointlist.h"
#include "View/pathwidget.h"
#include "colors.h"
#include <QMap>
#include <QVBoxLayout>
#include <QAbstractButton>
#include <QString>
#include <QStringList>
#include <QVector>


MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    points = std::shared_ptr<Points>(new Points(this));
    QWidget* mainWidget = new QWidget(this);

    QVBoxLayout* mainLayout = new QVBoxLayout(mainWidget);
    map = std::shared_ptr<Map>(new Map());

    QSettings settings;

    map->setMapFromFile(settings.value("mapFile", ":/maps/map.pgm").toString());

    /**************************************************************/

    map->setWidth(320);
    map->setHeight(608);
    map->setResolution(0.050000);
    map->setOrigin(Position(-1, -15.4));

    /**************************************************************/



    robots = std::shared_ptr<Robots>(new Robots());
    scene = new QGraphicsScene();

    graphicsView = new CustomQGraphicsView(scene, this);

    selectedRobot = NULL;
    scanningRobot = NULL;
    selectedPoint = NULL;
    editedPointView = NULL;
    updateRobotsThread = NULL;
    mapThread = NULL;

    /// Create the graphic item of the map
    QPixmap pixmap = QPixmap::fromImage(map->getMapImage());
    mapPixmapItem = new MapView(pixmap, QSize(geometry().width(), geometry().height()), map, this);


    /// Create the toolbar
    topLayout = new TopLayout(this);
    mainLayout->addWidget(topLayout);

    QHBoxLayout* bottom = new QHBoxLayout();

    initializePoints();

    pathPainter = std::shared_ptr<PathPainter>(new PathPainter(this, mapPixmapItem, points));

    initializeRobots();

    scene->addItem(mapPixmapItem);

    graphicsView->scale(std::max(graphicsView->parentWidget()->width()/scene->width(), graphicsView->parentWidget()->height()/scene->height()),
                        std::max(graphicsView->parentWidget()->width()/scene->width(), graphicsView->parentWidget()->height()/scene->height()));

    /// hides the scroll bars
    graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    graphicsView->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));


    leftMenu = new LeftMenu(this, points, robots, points, map, pathPainter);

    resetFocus();
    initializeLeftMenu();
    bottom->addWidget(leftMenu);

    rightLayout = new QVBoxLayout();
    bottom->addLayout(rightLayout);
    rightLayout->addWidget(graphicsView);


    initializeBottomPanel();

    graphicsView->show();

    /// to link the map and the home widget menu when a home is being edited
    connect(mapPixmapItem, SIGNAL(homeEdited(PointView*)), this, SLOT(homeEdited(PointView*)));

    /// to link the map and the point information menu when a point is being edited
    connect(mapPixmapItem, SIGNAL(newCoordinates(double, double)), this, SLOT(updateCoordinates(double, double)));

    /// to link the map and the path information when a path point is being edited
    connect(mapPixmapItem, SIGNAL(newCoordinatesPathPoint(double,double)), this, SLOT(updateEditedPathPoint(double, double)));

    /// to cancel the modifications on an edited point
    connect(leftMenu->getDisplaySelectedPoint()->getCancelButton(), SIGNAL(clicked(bool)), this, SLOT(cancelEvent()));

    /// to save the modifications on an edited point
    connect(leftMenu->getDisplaySelectedPoint()->getSaveButton(), SIGNAL(clicked(bool)), this, SLOT(updatePoint()));

    /// the purpose of this connection is just to propagate the signal to the map view through the main window
    connect(leftMenu->getDisplaySelectedPoint(), SIGNAL(nameChanged(QString, QString)), this, SLOT(updatePoint()));

    /// to reset the state of everybody when a user click on a random button while he was editing a point
    connect(leftMenu->getDisplaySelectedPoint(), SIGNAL(resetState(GraphicItemState, bool)),  this, SLOT(setGraphicItemsState(GraphicItemState, bool)));

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

    /// to know what message to display when a user is creating a path
    connect(mapPixmapItem, SIGNAL(newMessage(QString)), this, SLOT(setMessageCreationPath(QString)));

    /// to add a path point when we click on the map
    connect(mapPixmapItem, SIGNAL(addPathPoint(QString, double, double)), pathCreationWidget, SLOT(addPathPointSlot(QString, double, double)));

    /// to add a path point when we click on a pointView (which is relayed by the mainWindow)
    connect(this, SIGNAL(addPathPoint(QString, double, double)), pathCreationWidget, SLOT(addPathPointSlot(QString, double, double)));
    connect(this, SIGNAL(updatePathPainter()), pathPainter.get(), SLOT(updatePathPainterSlot()));
    connect(this, SIGNAL(updatePathPainterPointView()), pathPainter.get(), SLOT(updatePathPainterPointViewSlot()));
    connect(pathCreationWidget, SIGNAL(editTmpPathPoint(int, QString, double, double)), this, SLOT(editTmpPathPointSlot(int, QString, double, double)));
    connect(pathCreationWidget, SIGNAL(saveEditPathPoint()), this, SLOT(saveEditPathPointSlot()));
    connect(pathCreationWidget, SIGNAL(cancelEditPathPoint()), this, SLOT(cancelEditPathPointSlot()));
    connect(pathCreationWidget, SIGNAL(savePath()), this, SLOT(savePathSlot()));
    connect(this, SIGNAL(resetPath()), pathPainter.get(), SLOT(resetPathSlot()));
    connect(this, SIGNAL(resetPathCreationWidget()), pathCreationWidget, SLOT(resetWidget()));

    mainLayout->addLayout(bottom);
    graphicsView->setStyleSheet("CustomQGraphicsView{background-color: "+background_map_view+"}");
    setCentralWidget(mainWidget);

    /// to navigate with the tab key
    setTabOrder(leftMenu->getReturnButton(), pointsLeftWidget->getActionButtons()->getPlusButton());

    /// Centers the map
    centerMap();

    /// Some style
    //this->setStyleSheet("QWidget{background-color: white}");
    //topLayout->setAutoFillBackground(true);
    //topLayout->setStyleSheet("*{background-color: #5481a4}");
    this->setAutoFillBackground(true);
    rightLayout->setContentsMargins(0,0,0,0);
    bottom->setContentsMargins(0,0,0,0);
    mainLayout->setContentsMargins(0,0,0,0);
    this->setContentsMargins(0,0,0,0);
    mainWidget->setContentsMargins(0,0,0,0);
    topLayout->setContentsMargins(0,0,0,0);
    bottomLayout->setContentsMargins(0,0,0,0);
    mainLayout->setSpacing(0);
    //setStyleSheet("QPushButton{color: white}");


}

MainWindow::~MainWindow(){
    delete ui;
    if (updateRobotsThread != NULL && updateRobotsThread->isRunning() ) {
        updateRobotsThread->requestInterruption();
        updateRobotsThread->wait();
    }
    if (mapThread != NULL && mapThread->isRunning() ) {
        mapThread->requestInterruption();
        mapThread->wait();
    }
}

/**********************************************************************************************************************************/

//                                          ROBOTS and PATHS

/**********************************************************************************************************************************/


void MainWindow::initializeRobots(){

    /// Get the list of taken robot's name from the file
    QFile fileRead(ROBOTS_NAME_PATH);
    fileRead.open(QIODevice::ReadOnly);
    /// read the data serialized from the file
    QDataStream in(&fileRead);
    QMap<QString, QString> tmp;
    in >> tmp;
    robots->setRobotsNameMap(tmp);
    fileRead.close();


    updateRobotsThread = new UpdateRobotsThread(PORT_ROBOT_UPDATE);
    connect(updateRobotsThread, SIGNAL(robotIsAlive(QString, QString, QString, QString, int)), this, SLOT(robotIsAliveSlot(QString, QString, QString, QString, int)));
    updateRobotsThread->start();
    updateRobotsThread->moveToThread(updateRobotsThread);

/*

    QFile fileWrite(ROBOTS_NAME_PATH);
    fileWrite.resize(0);
    fileWrite.open(QIODevice::WriteOnly);
    QDataStream out(&fileWrite);
    QMap<QString, QString> tmpMap = robots->getRobotsNameMap();

    QString robotIp1 = "localhost";
    QString robotName1 = tmpMap.value(robotIp1, "Roboty");

    std::shared_ptr<Robot> robot1(new Robot(robotName1, robotIp1, this));
    robot1->setWifi("Swaghetti Yolognaise");
    RobotView* robotView1 = new RobotView(robot1, mapPixmapItem);
    connect(robotView1, SIGNAL(setSelectedSignal(RobotView*)), this, SLOT(setSelectedRobot(RobotView*)));
    robotView1->setPosition(200, 200);
    robotView1->setParentItem(mapPixmapItem);
    robots->add(robotView1);
    tmpMap[robot1->getIp()] = robot1->getName();

    QString robotIp2 = "192.168.4.12";
    QString robotName2 = tmpMap.value(robotIp2, "Roboto");
    std::shared_ptr<Robot> robot2(new Robot(robotName2, robotIp2, this));
    robot2->setWifi("Swaghetti Yolognaise");
    RobotView* robotView2 = new RobotView(robot2, mapPixmapItem);
    connect(robotView2, SIGNAL(setSelectedSignal(RobotView*)), this, SLOT(setSelectedRobot(RobotView*)));
    robotView2->setPosition(100, 100);
    robotView2->setParentItem(mapPixmapItem);
    robots->add(robotView2);
    tmpMap[robot2->getIp()] = robot2->getName();

    QString robotIp3 = "192.168.4.13";
    QString robotName3 = tmpMap.value(robotIp3, "Robota");
    std::shared_ptr<Robot> robot3(new Robot(robotName3, robotIp3, this));
    robot3->setWifi("Swaghetti Yolognaise");
    RobotView* robotView3 = new RobotView(robot3, mapPixmapItem);
    connect(robotView3, SIGNAL(setSelectedSignal(RobotView*)), this, SLOT(setSelectedRobot(RobotView*)));
    robotView3->setPosition(200, 300);
    robotView3->setParentItem(mapPixmapItem);
    robots->add(robotView3);
    tmpMap[robot3->getIp()] = robot3->getName();


    robots->setRobotsNameMap(tmpMap);
    out << robots->getRobotsNameMap();
    fileWrite.close();


    qDebug() << "RobotsNameMap on init" << robots->getRobotsNameMap();
    */
}

void MainWindow::updateRobot(const QString ipAddress, const float posX, const float posY, const float oriZ){

    float newPosX = (-map->getOrigin().getX()+posX)/map->getResolution() + ROBOT_WIDTH;
    float newPosY = map->getHeight()-(-map->getOrigin().getY()+posY)/map->getResolution()-ROBOT_WIDTH/2;
    float ori = asin(-oriZ) * 360.0 / PI + 90;

    RobotView* rv = robots->getRobotViewByIp(ipAddress);
    if(rv != NULL){
        rv->setPosition(newPosX, newPosY);
        rv->setOrientation(ori);
    } else {
        qDebug() << "(updateRobot) Could not find a RobotView for the robot at ip" << ipAddress;
    }
}

void MainWindow::connectToRobot(){
    bool checked = selectedRobotWidget->getScanBtn()->isChecked();
    qDebug() << "connectToRobot called" << checked;
    if(selectedRobot != NULL){
        if(checked){
            int ret = openConfirmMessage("Warning, scanning a new map will erase all previously created points, paths and selected home of robots");
            switch(ret){
                case QMessageBox::Cancel :
                    qDebug() << "clicked no";
                    selectedRobotWidget->getScanBtn()->setChecked(false);
                break;
                case QMessageBox::Ok :{
                    QString ip = selectedRobot->getRobot()->getIp();
                    qDebug() << "Trying to connect to : " << ip;

                    selectedRobot->getRobot()->resetCommandAnswer();
                    if(selectedRobot->getRobot()->sendCommand(QString("e"))){

                        selectedRobotWidget->getScanBtn()->setText("Stop to scan");
                        clearNewMap();
                        selectedRobotWidget->disable();
                        selectedRobotWidget->getScanBtn()->setEnabled(true);
                        setEnableAll(false, GraphicItemState::NO_EVENT);
                        scanningRobot = selectedRobot;

                        mapThread = new ScanMapThread(ip, PORT_MAP, MAP_PATH);
                        connect(mapThread, SIGNAL(valueChangedMap(QByteArray)),
                                this , SLOT(updateMap(QByteArray)));
                        connect(mapThread, SIGNAL(newScanSaved(QString)),
                                this , SLOT(sendNewMapToRobots(QString)));
                        mapThread->start();
                        mapThread->moveToThread(mapThread);

                        topLayout->setLabel(TEXT_COLOR_SUCCESS, "Scanning a new map");

                        selectedRobot->getRobot()->resetCommandAnswer();
                    } else {
                        selectedRobotWidget->getScanBtn()->setChecked(false);
                        topLayout->setLabel(TEXT_COLOR_DANGER, "Failed to start to scan a map, please try again");
                    }
                }
                break;
                default:
                    qDebug() << "MainWindow::connectToRobot should not be here";
                break;
            }
        } else {
            selectedRobot->getRobot()->resetCommandAnswer();
            if(selectedRobot->getRobot()->sendCommand(QString("f"))){
                qDebug() << "Need to wait for the last map ?";

                QString answer = selectedRobot->getRobot()->waitAnswer();
                QStringList answerList = answer.split(QRegExp("[ ]"), QString::SkipEmptyParts);
                if(answerList.size() > 1){
                    QString cmd = answerList.at(0);
                    bool success = (answerList.at(1).compare("done") == 0);
                    if((cmd.compare("f") == 0 && success) || answerList.at(0).compare("1") == 0){
                        qDebug() << "Stopped scanning the map";
                        selectedRobotWidget->getScanBtn()->setText("Scan a map");
                        selectedRobotWidget->enable();

                        hideAllWidgets();
                        selectedRobotWidget->setSelectedRobot(selectedRobot);
                        selectedRobotWidget->show();

                        setEnableAll(true);
                        topLayout->setLabel(TEXT_COLOR_SUCCESS, "Stopped scanning the map");
                    } else {
                        selectedRobotWidget->getScanBtn()->setChecked(true);
                        topLayout->setLabel(TEXT_COLOR_DANGER, "Failed to stop the scanning, please try again");
                    }
                }
                selectedRobot->getRobot()->resetCommandAnswer();

            } else {
                qDebug() << "Could not disconnect";
                selectedRobotWidget->getScanBtn()->setChecked(true);
                topLayout->setLabel(TEXT_COLOR_DANGER, "Failed to stop the scanning, please try again");
            }
        }
    } else {

        topLayout->setLabelDelay(TEXT_COLOR_DANGER, "You must first click a robot on the map to establish a connection",2500);

        qDebug() << "Select a robot first";
    }
}

void MainWindow::deletePath(int robotNb){
    qDebug() << "MainWindow::deletepath called on robot :" << robots->getRobotsVector().at(robotNb)->getRobot()->getName();
    std::shared_ptr<Robot> robot = robots->getRobotsVector().at(robotNb)->getRobot();
    if(robot->getPath().size() > 0){
        /// if the robot is not playing its path
        if(!robot->isPlayingPath()){
            msgBox.setIcon(QMessageBox::Question);
            int ret = openConfirmMessage("Are you sure you want to delete this path ?");
            switch (ret) {
                case QMessageBox::Ok:
                {
                    /// if the command is succesfully sent to the robot, we apply the change
                    robot->resetCommandAnswer();
                    if(robot->sendCommand(QString("k"))){
                        QString answer = robot->waitAnswer();
                        QStringList answerList = answer.split(QRegExp("[ ]"), QString::SkipEmptyParts);
                        if(answerList.size() > 1){
                            QString cmd = answerList.at(0);
                            bool success = (answerList.at(1).compare("done") == 0);
                            if((cmd.compare("k") == 0 && success) || answerList.at(0).compare("1") == 0){
                                clearPath(robotNb);
                                hideAllWidgets();
                                bottomLayout->getViewPathRobotBtnGroup()->button(robotNb)->setChecked(false);
                                bottomLayout->getViewPathRobotBtnGroup()->button(robotNb)->click();
                                topLayout->setLabel(TEXT_COLOR_SUCCESS, "Path deleted");

                            } else {
                                topLayout->setLabel(TEXT_COLOR_DANGER, "Failed to delete the path, please try again");
                            }
                        }
                        robot->resetCommandAnswer();
                    }
                }
                break;
                case QMessageBox::Cancel:
                    qDebug() << "Cancel was clicked";
                break;
                default:
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
                    std::shared_ptr<Robot> robot = robots->getRobotsVector().at(robotNb)->getRobot();
                    robot->resetCommandAnswer();
                    if(robot->sendCommand(QString("m"))){
                        QString answer = robot->waitAnswer();
                        QStringList answerList = answer.split(QRegExp("[ ]"), QString::SkipEmptyParts);
                        if(answerList.size() > 1){
                            QString cmd = answerList.at(0);
                            bool success = (answerList.at(1).compare("done") == 0);
                            if((cmd.compare("m") == 0 && success) || answerList.at(0).compare("1") == 0){
                                clearPath(robotNb);
                                hideAllWidgets();
                                bottomLayout->getViewPathRobotBtnGroup()->button(robotNb)->setChecked(false);
                                bottomLayout->getViewPathRobotBtnGroup()->button(robotNb)->click();
                                topLayout->setLabel(TEXT_COLOR_SUCCESS, "Path deleted");
                            } else {
                                topLayout->setLabel(TEXT_COLOR_DANGER, "Failed to delete the path, please try again");
                            }
                        }
                        robot->resetCommandAnswer();
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
    std::shared_ptr<Robot> robot = robots->getRobotsVector().at(robotNb)->getRobot();
    robot->resetCommandAnswer();
    if(robot->sendCommand(QString("l"))){
        QString answer = robot->waitAnswer();
        QStringList answerList = answer.split(QRegExp("[ ]"), QString::SkipEmptyParts);
        if(answerList.size() > 1){
            QString cmd = answerList.at(0);
            bool success = (answerList.at(1).compare("done") == 0);
            if((cmd.compare("l") == 0 && success) || answerList.at(0).compare("1") == 0){
                robot->setPlayingPath(false);
                bottomLayout->getPlayRobotBtnGroup()->button(robotNb)->setIcon(QIcon(":/icons/play.png"));
                bottomLayout->getStopRobotBtnGroup()->button(robotNb)->setEnabled(false);
                topLayout->setLabel(TEXT_COLOR_SUCCESS, "Path stopped");
            } else {
                topLayout->setLabel(TEXT_COLOR_DANGER, "Path failed to be stopped, please try again");
            }
        }
        robot->resetCommandAnswer();
    }
}

void MainWindow::playSelectedRobot(int robotNb){
    std::shared_ptr<Robot> robot = robots->getRobotsVector().at(robotNb)->getRobot();
    if(robot->isPlayingPath()){
        qDebug() << "pause path on robot " << robotNb << " : " << robot->getName();
        /// if the command is succesfully sent to the robot, we apply the change
        robot->resetCommandAnswer();
        if(robot->sendCommand(QString("d"))){
            QString answer = robot->waitAnswer();
            QStringList answerList = answer.split(QRegExp("[ ]"), QString::SkipEmptyParts);
            if(answerList.size() > 1){
                QString cmd = answerList.at(0);
                bool success = (answerList.at(1).compare("done") == 0);
                if((cmd.compare("d") == 0 && success) || answerList.at(0).compare("1") == 0){
                    robot->setPlayingPath(0);
                    bottomLayout->getPlayRobotBtnGroup()->button(robotNb)->setIcon(QIcon(":/icons/play.png"));
                    bottomLayout->getStopRobotBtnGroup()->button(robotNb)->setEnabled(true);
                    topLayout->setLabel(TEXT_COLOR_SUCCESS, "Path stopped");
                } else {
                    topLayout->setLabel(TEXT_COLOR_DANGER, "Path failed to be stopped, please try again");
                }
            }
            robot->resetCommandAnswer();
        }
    } else {
        qDebug() << "play path on robot " << robotNb << " : " << robot->getName();

        /// if the command is succesfully sent to the robot, we apply the change
        robot->resetCommandAnswer();
        if(robot->sendCommand(QString("j"))){
            qDebug() << "Let's wait";
            QString answer = robot->waitAnswer();
            qDebug() << "Done waiting";
            QStringList answerList = answer.split(QRegExp("[ ]"), QString::SkipEmptyParts);
            if(answerList.size() > 1){
                QString cmd = answerList.at(0);
                bool success = (answerList.at(1).compare("done") == 0);
                if((cmd.compare("j") == 0 && success) || answerList.at(0).compare("1") == 0){
                    robot->setPlayingPath(1);
                    bottomLayout->getPlayRobotBtnGroup()->button(robotNb)->setIcon(QIcon(":/icons/pause.png"));
                    bottomLayout->getStopRobotBtnGroup()->button(robotNb)->setEnabled(true);
                    topLayout->setLabel(TEXT_COLOR_SUCCESS, "Path playing");
                } else {
                    topLayout->setLabel(TEXT_COLOR_DANGER, "Path failed to start, please try again");
                }
            }
        }
    }
}

void MainWindow::viewPathSelectedRobot(int robotNb, bool checked){
    qDebug() << "MainWindow::viewPathSelectedRobot called" << robotNb << checked;
    if(checked){
        std::shared_ptr<Robot> robot = robots->getRobotsVector().at(robotNb)->getRobot();
        qDebug() << "viewPathSelectedRobot called on" << robot->getName();
        bottomLayout->uncheckViewPathSelectedRobot(robotNb);
        emit resetPath();

        for(int i = 0; i < robot->getPath().size(); i++){
            qDebug() << i << ":" << robot->getPath().at(i)->getPoint().getName()
                     << robot->getPath().at(i)->getPoint().getPosition().getX()
                     << robot->getPath().at(i)->getPoint().getPosition().getY()
                     << (int) robot->getPath().at(i)->getAction()
                     << robot->getPath().at(i)->getWaitTime();
        }
        pathPainter->setCurrentPath(robot->getPath());
        robot->setPath(pathPainter->getCurrentPath());
        bottomLayout->updateRobot(robotNb, robots->getRobotsVector().at(robotNb));
        emit updatePathPainter();
    } else {
        if(leftMenu->getDisplaySelectedPoint() && leftMenu->getDisplaySelectedPoint()->isVisible() && leftMenu->getDisplaySelectedPoint()->getPointView()->getPoint()->isPath()){
            leftMenu->getDisplaySelectedPoint()->getPointView()->hide();
            selectedPoint = NULL;
            leftMenu->getDisplaySelectedPoint()->setPointView(selectedPoint.get(), "");
            backEvent();
        }

        emit resetPath();
    }
}

void MainWindow::editSelectedRobot(RobotView* robotView){
    qDebug() << "editSelectedRobot robotview ";
    selectedRobot = robotView;
    robots->setSelected(robotView);

    hideAllWidgets();
    setGraphicItemsState(GraphicItemState::NO_EVENT);

    editSelectedRobotWidget->setSelectedRobot(selectedRobot);

    editSelectedRobotWidget->show();
    switchFocus(selectedRobot->getRobot()->getName(), editSelectedRobotWidget, MainWindow::WidgetType::ROBOT);

    leftMenu->getReturnButton()->setEnabled(false);
    leftMenu->getReturnButton()->setToolTip("Please save or discard your modifications before navigating the menu again.");
}


void MainWindow::setSelectedRobot(RobotView* robotView){

    qDebug() << "setSelectedRobot(RobotView* robotView)";
    leftMenu->show();

    hideAllWidgets();
    selectedRobot = robotView;
    robots->setSelected(robotView);
    selectedRobotWidget->setSelectedRobot(selectedRobot);
    selectedRobotWidget->show();
    switchFocus(robotView->getRobot()->getName(), selectedRobotWidget, MainWindow::WidgetType::ROBOT);


}

void MainWindow::robotBtnEvent(void){
    qDebug() << "robotBtnEvent called";
    leftMenuWidget->hide();
    robotsLeftWidget->show();
    switchFocus("Robots", robotsLeftWidget, MainWindow::WidgetType::ROBOTS);
}



void MainWindow::backSelecRobotBtnEvent(){
    qDebug() << "backSelecRobotBtnEvent called";
}

void MainWindow::editSelecRobotBtnEvent(){
    qDebug() << "editSelecRobotBtnEv"
                "ent called";
    editSelectedRobot(selectedRobot);
}

void MainWindow::addPathSelecRobotBtnEvent(){
    qDebug() << "MainWindow::addPathSelecRobotBtnEvent called on robot " << selectedRobot->getRobot()->getName();
    setMessageTop(TEXT_COLOR_INFO, "Click white points of the map to add new points to the path of " +
                  selectedRobot->getRobot()->getName() + "\nAlternatively you can click the \"+\" button to add an existing point to your path");

    /// stop displaying the currently displayed path if it exists

    /*int id = bottomLayout->getViewPathRobotBtnGroup()->checkedId();
    if(id != -1){
        viewPathSelectedRobot(id, false);
    }*/
    if(bottomLayout->getViewPathRobotBtnGroup()->checkedButton()){
        bottomLayout->getViewPathRobotBtnGroup()->checkedButton()->setChecked(false);
        pathCreationWidget->getPathPointList()->clear();
    }

    setEnableAll(false, GraphicItemState::CREATING_PATH, true, true);

    hideAllWidgets();
    pathCreationWidget->show();
    pathCreationWidget->updateRobot(selectedRobot->getRobot());

    switchFocus(selectedRobot->getRobot()->getName(), pathCreationWidget, MainWindow::WidgetType::ROBOT);


    /// hides the temporary pointview
    points->getTmpPointView()->hide();

    /// displays the points in order to make them available for the edition of the path,
    ///  we have to keep track of those which were hidden so we can hide them again once the edition is finished
    QMapIterator<QString, std::shared_ptr<QVector<std::shared_ptr<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();
        if(i.value()){
            for(int j = 0; j < i.value()->count(); j++){
                if(!i.value()->at(j)->isVisible() && i.value()->at(j)->getPoint()->getName().compare(TMP_POINT_NAME)){
                    i.value()->at(j)->show();
                    pointViewsToDisplay.push_back(i.value()->at(j));
                }
            }
        }
    }
}

void MainWindow::setSelectedRobotNoParent(QAbstractButton *button){
    resetFocus();
    setSelectedRobot(robots->getRobotViewByName(button->text()));
}

void MainWindow::setSelectedRobot(QAbstractButton *button){
    Q_UNUSED(button)
    qDebug() << "select a robot in robot group ";

    robotsLeftWidget->getActionButtons()->getEditButton()->setEnabled(true);
    robotsLeftWidget->getActionButtons()->getGoButton()->setEnabled(true);
    robotsLeftWidget->getActionButtons()->getMapButton()->setEnabled(true);
    RobotView* mySelectedRobot =  robots->getRobotViewByName(robotsLeftWidget->getBtnGroup()
                                                  ->getBtnGroup()->checkedButton()->text());
    robotsLeftWidget->getActionButtons()->getMapButton()->setChecked(mySelectedRobot->isVisible());
}

void MainWindow::selectViewRobot(){
    qDebug() << "select view robot  " <<robotsLeftWidget->getSelectedRobotName();
    setSelectedRobot(robots->getRobotViewByName(robotsLeftWidget->getSelectedRobotName()));
}

void MainWindow::setSelectedRobotFromPoint(){
    qDebug() << "setSelectedRobotFromPoint called : " << leftMenu->getDisplaySelectedPoint()->getRobotButton()->text();
    RobotView* rv = robots->getRobotViewByName(leftMenu->getDisplaySelectedPoint()->getRobotButton()->text());
    if(rv != NULL)
        setSelectedRobot(rv);
    else
        qDebug() << "setSelectedRobotFromPoint : something unexpected happened";
}

void MainWindow::backRobotBtnEvent(){
    qDebug() << "backRobotBtnEvent called";
    robotsLeftWidget->hide();
    leftMenuWidget->show();
}

void MainWindow::editRobotBtnEvent(){
    qDebug() << "editRobotBtnEvent called";

    editSelectedRobot(robots->getRobotViewByName(robotsLeftWidget->getBtnGroup()->getBtnGroup()->checkedButton()->text()));
}

void MainWindow::checkRobotBtnEventMenu(){
    qDebug() << "checkRobotBtnEventMenu called";
    QString name = robotsLeftWidget->getBtnGroup()->getBtnGroup()->checkedButton()->text();

    checkRobotBtnEvent(name);
}

void MainWindow::checkRobotBtnEventSelect(){
    qDebug() << "checkRobotBtnEventMenu called";
    QString name = selectedRobotWidget->getName();

    checkRobotBtnEvent(name);
}


void MainWindow::checkRobotBtnEvent(QString name){
    qDebug() << "checkRobotBtnEvent called" << name;


   RobotView* robotView =  robots->getRobotViewByName(name);
     robotView->display(!robotView->isVisible());
}

void MainWindow::cancelEditSelecRobotBtnEvent(){
    qDebug() << "cancelEditSelecRobotBtnEvent called";
    /// if the path has been changed, reset the path
    emit resetPathCreationWidget();
    emit resetPath();

    backEvent();

    leftMenu->getReturnButton()->setEnabled(true);
    leftMenu->getReturnButton()->setToolTip("");

    setEnableAll(true);
}

void MainWindow::robotSavedEvent(){
    qDebug() << "MainWindow::robotSavedEvent called";

    bool isOK = false;
    int change = 0;

    if ( editSelectedRobotWidget->getPathChanged())
    {
        change++;
        isOK = true;
    }
    /// if we changed the name
    if(selectedRobot->getRobot()->getName().compare(editSelectedRobotWidget->getNameEdit()->text()) != 0){
        qDebug() << "MainWindow::robotSavedEvent Name has been modified";
        selectedRobot->getRobot()->resetCommandAnswer();
        if(selectedRobot->getRobot()->sendCommand(QString("a \"") + editSelectedRobotWidget->getNameEdit()->text() + "\"")){
            QString answer = selectedRobot->getRobot()->waitAnswer();
            QStringList answerList = answer.split(QRegExp("[ ]"), QString::SkipEmptyParts);
            if(answerList.size() > 1){
                QString cmd = answerList.at(0);
                bool success = (answerList.at(1).compare("done") == 0);
                if((cmd.compare("a") == 0 && success) || answerList.at(0).compare("1") == 0){
                    isOK = true;
                    change++;

                    QMap<QString, QString> tmp = robots->getRobotsNameMap();
                    tmp[selectedRobot->getRobot()->getIp()] = editSelectedRobotWidget->getNameEdit()->text();
                    emit changeCmdThreadRobotName(editSelectedRobotWidget->getNameEdit()->text());
                    robots->setRobotsNameMap(tmp);

                    QFile fileWrite(ROBOTS_NAME_PATH);
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
            selectedRobot->getRobot()->resetCommandAnswer();
        }
    }

    /// if we changed the wifi
    if (editSelectedRobotWidget->getWifiPwdEdit()->text() != "......"){
        qDebug() << "MainWindow::robotSavedEvent Wifi has been modified";
        selectedRobot->getRobot()->resetCommandAnswer();
        if(selectedRobot->getRobot()->sendCommand(QString("b \"")
                  + editSelectedRobotWidget->getWifiNameEdit()->text() + "\" \""
                  + editSelectedRobotWidget->getWifiPwdEdit()->text() + "\"")){

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
            selectedRobot->getRobot()->resetCommandAnswer();
        }
    }

    /// if we changed the home
    std::shared_ptr<PointView> pointView = editSelectedRobotWidget->getHome();
    if(pointView != NULL && (selectedRobot->getRobot()->getHome() == NULL || !(&(*(pointView->getPoint())) == &(*(selectedRobot->getRobot()->getHome()->getPoint()))))){
        qDebug() << "MainWindow::robotSavedEvent Home has been modified";
        bool done = false;

        if(pointView->getPoint()->isTemporary()){
            qDebug() << "MainWindow::robotSavedEvent Tmp point";
            pointView->getPoint()->setHome(Point::PointType::HOME, selectedRobot->getRobot()->getName());

            points->addPoint(NO_GROUP_NAME, points->getGroups()->value(TMP_GROUP_NAME)->takeFirst());
            points->addTmpPoint(mapPixmapItem, this);

            XMLParser parserPoints(XML_PATH);
            parserPoints.save(*points);
            done = true;
        } else {
            qDebug() << "MainWindow::robotSavedEvent Permanent point";
            if(pointView->getPoint()->setHome(Point::PointType::HOME, selectedRobot->getRobot()->getName())){
                XMLParser parserPoints(XML_PATH);
                parserPoints.save(*points);
                done = true;
            } else {
                setMessageTop(TEXT_COLOR_DANGER, "Sorry, this point is already a home\nPlease select another");
            }
        }
        pointsLeftWidget->updateGroupButtonGroup();


        if(done){
            if(selectedRobot->getRobot()->getHome() != NULL)
                selectedRobot->getRobot()->getHome()->getPoint()->setHome(Point::PointType::PERM, "");
            selectedRobot->getRobot()->setHome(editSelectedRobotWidget->getHome());
        }
        isOK = true;
        change++;
    }

    /// if we changed the path
    if(editSelectedRobotWidget->getPathChanged()){
        qDebug() << "MainWindow::robotSavedEvent path changed";
        std::shared_ptr<Robot> robot = selectedRobot->getRobot();
        QString pathStr = "";

        qDebug() << "MainWindow::robotSavedEvent" << pathPainter->getCurrentPath().size();
        for(int i = 0; i < pathPainter->getCurrentPath().size(); i++){
            std::shared_ptr<PathPoint> pathPoint = pathPainter->getCurrentPath().at(i);
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
        robot->resetCommandAnswer();
        if(robot->sendCommand(QString("i ") + pathStr)){
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
        if(change > 0){
            if (isOK){
                if(editSelectedRobotWidget->getPathChanged()){
                    selectedRobot->getRobot()->setPath(pathPainter->getCurrentPath());

                    int id = robots->getRobotId(selectedRobot->getRobot()->getName());
                    bottomLayout->updateRobot(id, selectedRobot);
                    bottomLayout->getViewPathRobotBtnGroup()->button(id)->setChecked(true);
                    viewPathSelectedRobot(id, true);

                }

                editSelectedRobotWidget->setPathChanged(false);

                backEvent();
                leftMenu->getReturnButton()->setEnabled(true);
                leftMenu->getReturnButton()->setToolTip("");

                editSelectedRobotWidget->editName();

                setGraphicItemsState(GraphicItemState::NO_STATE);

                robotsLeftWidget->updateRobots(robots);
                bottomLayout->updateRobot(robots->getRobotId(selectedRobot->getRobot()->getName()), selectedRobot);

                selectedRobotWidget->setSelectedRobot(selectedRobot);
                editSelectedRobotWidget->setSelectedRobot(selectedRobot);
                setEnableAll(true);

                setMessageTop(TEXT_COLOR_SUCCESS, "Robot successfully edited");
                qDebug() << "Robot successfully edited";
            }
        } else {
            setMessageTop(TEXT_COLOR_INFO, "Nothing has been modified because this point was already the home point of the robot " + selectedRobot->getRobot()->getName());
            qDebug() << "Nothing has been modified because this point was already the home point of the robot " + selectedRobot->getRobot()->getName();
        }
    }
}


void MainWindow::editTmpPathPointSlot(int id, QString name, double x, double y){
    qDebug() << "MainWindow::editTmpPathPointSlot called : " << id << name << x << y;

    editedPointView = points->getGroups()->value(PATH_GROUP_NAME)->at(id);

    setMessageTop(TEXT_COLOR_INFO, "Drag the selected point or click the map and click \"Save changes\" to modify the path of your robot");
    leftMenu->setEnableReturnCloseButtons(false);

    if(editedPointView == NULL){
        qDebug() << "MainWindow::editTmpPathPointSlot Error : No pointview found to edit";
    } else {
        qDebug() << "MainWindow::editTmpPathPointSlot Pointview found";

        int nbWidget = pathPainter->nbUsedPointView(name, x ,y);
        qDebug() << "MainWindow::editTmpPathPointSlot number of widget with the same pointView : " << nbWidget;
        /// if 2 path points have the same pointView, we need to create a copy to only
        /// move one of the two path points
        if(nbWidget > 1){
            editedPointView = points->createPoint(
                        editedPointView->getPoint()->getName(),
                        editedPointView->getPoint()->getPosition().getX(),
                        editedPointView->getPoint()->getPosition().getY(),
                        true, Point::PointType::PATH,
                        mapPixmapItem, this);
            points->getGroups()->value(PATH_GROUP_NAME)->remove(id);
            points->insertPoint(PATH_GROUP_NAME, id, editedPointView);
        }

        setGraphicItemsState(GraphicItemState::NO_EVENT, false);
        mapPixmapItem->setState(GraphicItemState::EDITING_PATH);
        editedPointView->setFlag(QGraphicsItem::ItemIsMovable);
        editedPointView->setState(GraphicItemState::EDITING_PATH);
        /// set by hand in order to keep the colors consistent while editing the point and after
        editedPointView->setPixmap(PointView::PixmapType::HOVER);
    }
}

void MainWindow::savePathSlot(){
    qDebug() << "MainWindow::savePath called";
    /// we hide the points that we displayed for the edition of the path
    for(int i = 0; i < pointViewsToDisplay.size(); i++)
        pointViewsToDisplay.at(i)->hide();
    pointViewsToDisplay.clear();


    backEvent();
    setGraphicItemsState(GraphicItemState::NO_EVENT);
    editSelectedRobotWidget->setPathChanged(true);
    editSelectedRobotWidget->setPath(pathPainter->getCurrentPath());
    emit updatePathPainter();
}

void MainWindow::cancelPathSlot(){
    qDebug() << "MainWindow::cancelPathSlot called";
    /// we hide the points that we displayed for the edition of the path

    for(int i = 0; i < pointViewsToDisplay.size(); i++)
        pointViewsToDisplay.at(i)->hide();
    pointViewsToDisplay.clear();

    emit resetPathCreationWidget();
    emit resetPath();
    pathPainter->setCurrentPath(selectedRobot->getRobot()->getPath());
    selectedRobot->getRobot()->setPath(pathPainter->getCurrentPath());
    bottomLayout->updateRobot(robots->getRobotId(selectedRobot->getRobot()->getName()), selectedRobot);

    backEvent();
}

void MainWindow::addPointPathSlot(QString name, double x, double y){
    qDebug() << "addPathPoint called on point via * point" << x << y;
    /// Relay to pathPainter::addPathPointSlot()
    emit addPathPoint(name, x, y);
}

void MainWindow::saveEditPathPointSlot(void){
    qDebug() << "MainWindow::saveEditPathPointSlot called";

    setEnableAll(false, GraphicItemState::CREATING_PATH, false, true);
    pathPainter->updateCurrentPath();

    editedPointView->setFlag(QGraphicsItem::ItemIsMovable, false);
    editedPointView = NULL;
}

void MainWindow::cancelEditPathPointSlot(void){
    qDebug() << "MainWindow::cancelEditPathPointSlot called";

    setEnableAll(false, GraphicItemState::CREATING_PATH, false, true);

    int id = pathCreationWidget->getPathPointList()->row(pathCreationWidget->getPathPointList()->currentItem());
    Position pos = pathPainter->getCurrentPath().at(id)->getPoint().getPosition();
    editedPointView->setPos(pos.getX(), pos.getY());

    emit updatePathPainter();

    editedPointView->setFlag(QGraphicsItem::ItemIsMovable, false);
    editedPointView = NULL;
}

void MainWindow::updatePathPainterPointViewSlot(){
    /// Relay to pathPainter::updatePathPainterSlot()
    emit updatePathPainterPointView();
}

void MainWindow::editHomeEvent(){
    qDebug() << "editHomeEvent called";
    if(editSelectedRobotWidget->getNameEdit()->isEnabled()){
        setMessageTop(TEXT_COLOR_INFO, "Click on the map or on a point to select a home for the robot " + selectedRobot->getRobot()->getName());
        editSelectedRobotWidget->getHomeBtn()->setText("Cancel");
        editSelectedRobotWidget->disableAll();
        editSelectedRobotWidget->getHomeBtn()->setEnabled(true);
        setEnableAll(false, GraphicItemState::EDITING_HOME);

    } else {
        setMessageTop(TEXT_COLOR_NORMAL,"");
        if(selectedRobot->getRobot()->getHome() != NULL){
            editSelectedRobotWidget->getHomeBtn()->setText(selectedRobot->getRobot()->getHome()->getPoint()->getName());
        } else {
            editSelectedRobotWidget->getHomeBtn()->setText("Add home");
        }
        editSelectedRobotWidget->enableAll();
        setEnableAll(true);
    }
}

void MainWindow::homeEdited(PointView* pointView){
    qDebug() << "MainWindow::homeEdited called" << pointView->getPoint()->getName();

    editSelectedRobotWidget->setHome(std::shared_ptr<PointView>(pointView));
    editSelectedRobotWidget->getHomeBtn()->setText("Edit home");

    if (pointView->getPoint()->getName().compare(TMP_POINT_NAME) == 0){
        QString name = QString::number(pointView->getPoint()->getPosition().getX(),'f', 1) + "; " + QString::number(pointView->getPoint()->getPosition().getY(),'f', 1);
        editSelectedRobotWidget->getHomeLabel()->setText("Home: " + name);
    } else {
        editSelectedRobotWidget->getHomeLabel()->setText("Home: " + pointView->getPoint()->getName());
    }

    editSelectedRobotWidget->enableAll();
    setEnableAll(true, GraphicItemState::NO_EVENT);
}

void MainWindow::showHome(){
    qDebug() << "MainWindow::showHome called" << (selectedRobot->getRobot()->getHome()==NULL);
    points->setPixmapAll(QPixmap(PIXMAP_NORMAL));

    if(selectedRobot->getRobot()->getHome() != NULL){
        std::shared_ptr<PointView> pointView = selectedRobot->getRobot()->getHome();
        pointView->setPixmap(PointView::PixmapType::NORMAL);
        if(pointView->isVisible()){
            qDebug() << "home is visible";
            pointView->setWasShown(true);
        }else{
            qDebug() << "home is not visible";
            pointView->setWasShown(false);
        }

        pointView->show();
    }

    RobotView* robotView =  robots->getRobotViewByName(selectedRobot->getRobot()->getName());
    /// If the robot has a path, we display it, otherwise we show the button to add the path
    if(robotView->getRobot()->getPath().size() > 0){
        //qDebug() << "MainWindow::showHome I have a path !";
        selectedRobotWidget->getPathWidget()->setPath(robotView->getRobot()->getPath());
        selectedRobotWidget->getPathWidget()->show();
        selectedRobotWidget->getNoPath()->hide();

        editSelectedRobotWidget->setPath(robotView->getRobot()->getPath());
    } else {
        qDebug() << "MainWindow::showHome I don't have a path !";
        selectedRobotWidget->getPathWidget()->hide();
        selectedRobotWidget->getNoPath()->show();

        editSelectedRobotWidget->clearPath();
    }
}

void MainWindow::clearPath(const int robotNb){
    qDebug() << "MainWindow::clearPath called";
    if(robots->getRobotsVector().at(robotNb)->getRobot()->isPlayingPath()){
        qDebug() << "MainWindow::clearPath pause path on robot before supp " << robotNb << " : " << robots->getRobotsVector().at(robotNb)->getRobot()->getName();
        robots->getRobotsVector().at(robotNb)->getRobot()->setPlayingPath(0);
        bottomLayout->getPlayRobotBtnGroup()->button(robotNb)->setIcon(QIcon(":/icons/play.png"));
    }
    robots->getRobotsVector().at(robotNb)->getRobot()->getPath().clear();

    bottomLayout->deletePath(robotNb);
}

void MainWindow::hideHome(void){
    qDebug() << "MainWindow::hideHome called";

    if(selectedRobot->getRobot()->getHome() != NULL){
        std::shared_ptr<PointView> pointView = selectedRobot->getRobot()->getHome();
        qDebug() << "Home was shown :" << pointView->getWasShown();
        if(!pointView->getWasShown()){
            qDebug() << "hidding the home";
            pointView->hide();
        }
    }
}

void MainWindow::goHomeBtnEvent(){
    qDebug() << "go home robot " << selectedRobot->getRobot()->getName() << (selectedRobot->getRobot()->getHome() == NULL);
    /// TODO change to go from the point saved in the robot files
    float oldPosX = selectedRobot->getRobot()->getHome()->getPoint()->getPosition().getX();
    float oldPosY = selectedRobot->getRobot()->getHome()->getPoint()->getPosition().getY();

    float newPosX = (oldPosX - ROBOT_WIDTH) * map->getResolution() + map->getOrigin().getX();
    float newPosY = (-oldPosY + map->getHeight() - ROBOT_WIDTH/2) * map->getResolution() + map->getOrigin().getY();
    qDebug() << "Go to next point :" << newPosX << newPosY;
    int waitTime = -1;

    /// if the command is succesfully sent to the robot, we apply the change
    selectedRobot->getRobot()->resetCommandAnswer();
    if(selectedRobot->getRobot()->sendCommand(QString("c \"") + QString::number(newPosX) + "\" \""  + QString::number(newPosY) + "\" \""  + QString::number(waitTime) + "\"")){
        QString answer = selectedRobot->getRobot()->waitAnswer();
        QStringList answerList = answer.split(QRegExp("[ ]"), QString::SkipEmptyParts);
        if(answerList.size() > 1){
            QString cmd = answerList.at(0);
            bool success = (answerList.at(1).compare("done") == 0);
            if((cmd.compare("d") == 0 && success) || answerList.at(0).compare("1") == 0){
                qDebug() << "Going home";
                topLayout->setLabel(TEXT_COLOR_SUCCESS, "Robot going home");
            } else {
                topLayout->setLabel(TEXT_COLOR_DANGER, "Failed to tell the robot to go home, please try again");
            }
        }
        selectedRobot->getRobot()->resetCommandAnswer();
    }
}

void MainWindow::robotIsAliveSlot(QString hostname, QString ip, QString mapId, QString ssid, int stage){
    QRegExp rx("[0-9]{1,3}\\.[0-9]{1,3}\\.[0-9]{1,3}\\.[0-9]{1,3}");
    rx.indexIn(ip);
    ip = rx.cap(0);
    RobotView* rv = robots->getRobotViewByIp(ip);

    if(rv != NULL){
        qDebug() << "Robot" << hostname << "at ip" << ip << "is still alive and has the map id :" << mapId;
        rv->getRobot()->ping();
        /// TODO see for changes (battery)

    } else {
        qDebug() << "Robot" << hostname << "at ip" << ip << "just connected and has the map id :" << mapId;

        std::shared_ptr<Robot> robot(new Robot(hostname, ip, this));
        robot->setWifi(ssid);
        rv = new RobotView(robot, mapPixmapItem);
        connect(rv, SIGNAL(setSelectedSignal(RobotView*)), this, SLOT(setSelectedRobot(RobotView*)));
        rv->setPosition(robots->getRobotsVector().count()*100+100, robots->getRobotsVector().count()*100+100);
        rv->setParentItem(mapPixmapItem);
        robots->add(rv);
        bottomLayout->addRobot(rv);
        robotsLeftWidget->updateRobots(robots);

        /// Check if connection by usb
        if(ip.endsWith(".7.1") || ip.endsWith(".7.2") || ip.endsWith(".7.3")){
            selectedRobot = rv;
            switchFocus(hostname, editSelectedRobotWidget, MainWindow::WidgetType::ROBOT);
            editSelectedRobotWidget->setSelectedRobot(selectedRobot, true);
            hideAllWidgets();
            editSelectedRobotWidget->show();
            leftMenu->show();
            setEnableAll(false, GraphicItemState::NO_EVENT);
        } else {
            QMap<QString, QString> tmp = robots->getRobotsNameMap();
            tmp[ip] = hostname;
            robots->setRobotsNameMap(tmp);

            QFile fileWrite(ROBOTS_NAME_PATH);
            fileWrite.resize(0);
            fileWrite.open(QIODevice::WriteOnly);
            QDataStream out(&fileWrite);
            out << robots->getRobotsNameMap();
            fileWrite.close();
            qDebug() << "RobotsNameMap updated" << robots->getRobotsNameMap();
        }
    }

    /// Check if the robot has the current map
    QSettings settings;
    QString currMapId = settings.value("mapId", "{00000000-0000-0000-0000-000000000000}").toString();
    if(mapId.compare(currMapId) == 0){
        qDebug() << "Which is the current map";
    } else {
        qDebug() << "Which is an old map";
        sendNewMapToRobot(rv->getRobot(), currMapId);
    }
/*
    /// To put in a different color the stage where the robot is currently at
    if(rv->getRobot()->isPlayingPath()){
        QString greenPart = QString("<span style=" color:#00ff00;">%1</span>").arg(bottomLayout->getVectorPathLabel().at(robotId)->text().mid(5));
        bottomLayout->getVectorPathLabel().at(robotId)
    }
*/
    /// Check the current stage of the robot
    int robotId = robots->getRobotId(rv->getRobot()->getName());
    if(rv->getRobot()->isPlayingPath() && rv->getRobot()->getPath().size() == stage){
        setMessageTop(TEXT_COLOR_SUCCESS, "The robot " + rv->getRobot()->getName() + " has successfully reached its destination");
        bottomLayout->getPlayRobotBtnGroup()->button(robotId)->setIcon(QIcon(":/icons/play.png"));
        bottomLayout->getStopRobotBtnGroup()->button(robotId)->setEnabled(false);
    }
}

void MainWindow::robotIsDeadSlot(QString hostname,QString ip){
    qDebug() << "Robot" << hostname << "at ip" << ip << "... He is dead, Jim!!";
    setMessageTop(TEXT_COLOR_DANGER, QString("Robot " + hostname + " at ip " + ip +" disconnected."));

    qDebug() << "Robots IP";
    for(int i = 0; i < robots->getRobotsVector().size(); i++){
        qDebug() << robots->getRobotsVector().at(i)->getRobot()->getIp();
    }

    RobotView* rv = robots->getRobotViewByIp(ip);
    int id = robots->getRobotId(hostname);
    qDebug() << "Dead robot's id :" << id;

    if(rv != NULL && rv->getRobot() != NULL){
        /// we stop the robots threads
        rv->getRobot()->stopThreads();
        qDebug() << "robots threads cleaned";

        /// if the robot had a home, make the point a normal point
        if(rv->getRobot()->getHome() != NULL)
            rv->getRobot()->getHome()->getPoint()->setHome(Point::PointType::PERM, "");
        qDebug() << "home cleaned";

        /// if selected => if one of this robot related menu is open
        if(selectedRobot != NULL && selectedRobot->getRobot()->getIp().compare(ip) == 0){
            if(editSelectedRobotWidget->isVisible()){
                setGraphicItemsState(GraphicItemState::NO_STATE);
            }

            /// if a box to save/edit this robot is open
            if(msgBox.isVisible()){
                msgBox.close();
                qDebug() << "Closed the msgBox";
            } else {
                qDebug() << "No msgBox to clean";
            }

            hideAllWidgets();
            leftMenu->hide();
            qDebug() << "selectedRobot cleaned";
        }

        /// if the robot is scanning
        if(scanningRobot != NULL && scanningRobot->getRobot()->getIp().compare(ip) == 0){
            selectedRobotWidget->getScanBtn()->setChecked(false);
            selectedRobotWidget->getScanBtn()->setText("Scan a map");
            selectedRobotWidget->enable();

            hideAllWidgets();
            setEnableAll(true);
            qDebug() << "scanningRobot cleaned";
        }

        /// delete robotview
        scene->removeItem(rv);
        qDebug() << "scene cleaned";

        /// enlever de robots
        robots->remove(rv);
        qDebug() << "robots cleaned";

        /// update robotsLeftWidget
        robotsLeftWidget->updateRobots(robots);
        qDebug() << "robotsLeftWidget cleaned";

        /// bottomLayout
        bottomLayout->removeRobot(id);
        qDebug() << "bottomLayout cleaned";

        qDebug() << "Cleaning of the deleted robot finished";
        setMessageTop(TEXT_COLOR_DANGER, QString("Robot " + hostname + " at ip " + ip +" disconnected."));
    } else {
        qDebug() << "(robotIsDeadSlot) A problem occured, the RobotView or its Robot are NULL";
    }
}

void MainWindow::setMessageCreationPath(QString message){
    setMessageTop(TEXT_COLOR_DANGER, message);
    delay(2500);
    setMessageTop(TEXT_COLOR_INFO, "Click white points of the map to add new points to the path of " +
                  selectedRobot->getRobot()->getName() + "\nAlternatively you can click the \"+\" button to add an existing point to your path");
}

void MainWindow::updateEditedPathPoint(double x, double y){
    qDebug() << "MainWindow::updateEditedPathPoint called";

    if(editedPointView)
        editedPointView->setPos(x, y);
    else
        qDebug() << "MainWindow::updateEditedPathPoint Could not find the pointView to edit";


    emit updatePathPainter();

    if(map->getMapImage().pixelColor(x, y).red() >= 254){
        setMessageTop(TEXT_COLOR_INFO, "You can click either \"Save changes\" to modify your path permanently or \"Cancel\" to keep the original path. If you want you can keep editing your point");
        static_cast<PathPointCreationWidget*> (pathCreationWidget->getPathPointList()->itemWidget(pathCreationWidget->getPathPointList()->currentItem())) -> getSaveEditBtn()->setEnabled(true);
    } else {
        setMessageTop(TEXT_COLOR_DANGER, "You cannot save the current path because the point that you are editing is not in a known area of the map");
        static_cast<PathPointCreationWidget*> (pathCreationWidget->getPathPointList()->itemWidget(pathCreationWidget->getPathPointList()->currentItem())) -> getSaveEditBtn()->setEnabled(false);
    }
}

void MainWindow::moveEditedPathPointSlot(void){
    qDebug() << "MainWindow::moveEditedPathPointSlot called";
    emit updatePathPainter();

    if(map->getMapImage().pixelColor(editedPointView->getPoint()->getPosition().getX(), editedPointView->getPoint()->getPosition().getY()).red() >= 254){
        setMessageTop(TEXT_COLOR_INFO, "You can click either \"Save changes\" to modify your path permanently or \"Cancel\" to keep the original path. If you want you can keep editing your point");
        static_cast<PathPointCreationWidget*> (pathCreationWidget->getPathPointList()->itemWidget(pathCreationWidget->getPathPointList()->currentItem())) -> getSaveEditBtn()->setEnabled(true);
    } else {
        setMessageTop(TEXT_COLOR_DANGER, "You cannot save the current path because the point that you are editing is not in a known area of the map");
        static_cast<PathPointCreationWidget*> (pathCreationWidget->getPathPointList()->itemWidget(pathCreationWidget->getPathPointList()->currentItem())) -> getSaveEditBtn()->setEnabled(false);
    }
}

void MainWindow::sendNewMapToRobots(QString ipAddress){
    qDebug() << "sendNewMapToRobots called";
    /// We create a unique ID for the map
    QUuid mapId = QUuid::createUuid();
    qDebug() << "New map id :" << mapId.toString();
    QVector<RobotView*> robotsVector = robots->getRobotsVector();

    /// Save the map id in settings
    QSettings settings;
    settings.setValue("mapId", mapId.toString());

    /// We send the map to each robot
    for(int i = 0; i < robotsVector.size(); i++){
        std::shared_ptr<Robot> robot = robotsVector.at(i)->getRobot();

        /// No need to send the map to the robot that scanned it
        // TODO remettre/enlever comment

        if(robot->getIp().compare(ipAddress) != 0){
            qDebug() << "Sending the map to" << robot->getName() << "at ip" << robot->getIp();
            sendNewMapToRobot(robot, mapId.toString());
        } else {
            qDebug() << "The robot" << robot->getName() << "at ip" << robot->getIp() << "already has the current map";
        }
    }
    qDebug() << "Sent the map to the robots";
}

void MainWindow::sendNewMapToRobot(std::shared_ptr<Robot> robot, QString mapId){
    qDebug() << "sendNewMapToRobot called on" << robot->getName() << "at ip" << robot->getIp() << "sending map id :" << mapId;

    qDebug() << "Setting the map ID to the robot" << robot->getName();
    robot->setMapId(QUuid(mapId));

    qDebug() << "Sending the new map id to the robot" << robot->getName();
    /// Push the map id to send
    QByteArray byteArray;
    byteArray.push_back(mapId.toUtf8());
    byteArray.push_back(';');

    /// Push the map metadata to send
    QString mapMetadata = QString::number(map->getWidth()) + ' ' + QString::number(map->getHeight()) +
            ' ' + QString::number(map->getResolution()) + ' ' + QString::number(map->getOrigin().getX()) +
            ' ' + QString::number(map->getOrigin().getY());

    byteArray.push_back(mapMetadata.toUtf8());
    byteArray.push_back(';');

    /// Push the map to send
    QFile file(MAP_PATH);
    if (!file.open(QIODevice::ReadOnly)) return;
    QByteArray blob = file.readAll();
    qDebug() << "Followed by the new map of size" << blob.size();
    byteArray.push_back(blob);

    robot->sendNewMap(byteArray);
    qDebug() << "Done sending the new map";
}


void MainWindow::updateAllPaths(void){
    for(int i = 0; i < robots->getRobotsVector().size(); i++){
        std::shared_ptr<Robot> robot = robots->getRobotsVector().at(i)->getRobot();
        QVector<std::shared_ptr<PathPoint>> path = robot->getPath();
        for(int j = 0; j < path.size(); j++){
            Point point = path.at(j)->getPoint();
            qDebug() << "MainWindow::updatePoint Before" << point.getName();
            if(!points->isAPoint(point.getName(), point.getPosition().getX(), point.getPosition().getY())){
                point.setName(PATH_POINT_NAME + QString::number(j+1));
                path.at(j)->setPoint(point);
            }
            qDebug() << "MainWindow::updatePoint After" << point.getName();
        }
        robot->setPath(path);
        bottomLayout->updateRobot(robots->getRobotId(robot->getName()), robots->getRobotsVector().at(i));
    }
}

/**********************************************************************************************************************************/

//                                          MAPS

/**********************************************************************************************************************************/

void MainWindow::updateMetadata(const int width, const int height, const float resolution,
                                const float originX, const float originY){
    if(width != map->getWidth())
        map->setWidth(width);

    if(height != map->getHeight())
        map->setHeight(height);

    if(resolution != map->getResolution())
        map->setResolution(resolution);

    if(originX != map->getOrigin().getX() || originY != map->getOrigin().getY())
        map->setOrigin(Position(originX, originY));

    qDebug() << "Map metadata updated : " << map->getWidth() << " " << map->getHeight() << " "
             << map->getResolution() << " " << map->getOrigin().getX()  << " " << map->getOrigin().getY() ;
}

void MainWindow::updateMap(const QByteArray mapArray){
    map->setMapFromArray(mapArray);
    QPixmap pixmap = QPixmap::fromImage(map->getMapImage());
    mapPixmapItem->setPixmap(pixmap);
    scene->update();
}

void MainWindow::saveMapBtnEvent(){
    qDebug() << "saveMapBtnEvent called";

    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
            "", tr("Images (*.pgm)"));
    qDebug() << "FileName :" <<  fileName;

    if(fileName != ""){
        fileName += ".pgm";
        QSettings settings;
        settings.setValue("mapFile", fileName);

        map->saveToFile(fileName);
    } else {
        qDebug() << "Please select a file";
    }
}

void MainWindow::loadMapBtnEvent(){
    qDebug() << "loadMapBtnEvent called";
    int ret = openConfirmMessage("Warning, loading a new map will erase all previously created points, paths and selected home of robots");
    switch(ret){
        case QMessageBox::Cancel :
            qDebug() << "clicked no";
        break;
        case QMessageBox::Ok :{
            QString fileName = QFileDialog::getOpenFileName(this,
                tr("Open Image"), "", tr("Image Files (*.pgm)"));
            qDebug() << "File name :" << fileName;

            if(fileName != ""){
                QSettings settings;
                settings.setValue("mapFile", fileName);

                clearNewMap();
                map->setMapFromFile(fileName);
                QPixmap pixmap = QPixmap::fromImage(map->getMapImage());
                mapPixmapItem->setPixmap(pixmap);
                scene->update();
                centerMap();
            }
        }
        break;
        default:
            qDebug() << "MainWindow::loadMapBtnEvent should never be here";
        break;
    }
}

void MainWindow::backMapBtnEvent(){
    qDebug() << "backMapBtnEvent called";
    mapLeftWidget->hide();
    leftMenuWidget->show();
}

void MainWindow::mapBtnEvent(){
    qDebug() << "mapBtnEvent called";
    leftMenuWidget->hide();
    mapLeftWidget->show();
}

/**********************************************************************************************************************************/

//                                          MENUS

/**********************************************************************************************************************************/


void MainWindow::initializeLeftMenu(){
    lastWidgets =  QList<QPair<QPair<QWidget*,QString>, MainWindow::WidgetType>>();
    leftMenuWidget = leftMenu->getLeftMenuWidget();
    pointsLeftWidget = leftMenu->getPointsLeftWidget();
    selectedRobotWidget = leftMenu->getSelectedRobotWidget();
    robotsLeftWidget = leftMenu->getRobotsLeftWidget();
    mapLeftWidget = leftMenu->getMapLeftWidget();
    editSelectedRobotWidget = leftMenu->getEditSelectedRobotWidget();
    selectedPointWidget = leftMenu->getSelectedPointWidget();
    createPointWidget = leftMenu->getEditSelectedPointWidget();
    pathCreationWidget = leftMenu->getPathCreationWidget();
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
    leftMenu->hide();
    setEnableAll(true);
    if(leftMenu->getDisplaySelectedPoint()->getPointView()){
        std::shared_ptr<PointView> displaySelectedPointView = points->findPointView(leftMenu->getDisplaySelectedPoint()->getPointName());
        if(displaySelectedPointView)
           displaySelectedPointView->setPixmap(PointView::PixmapType::NORMAL);
    }
}

/**********************************************************************************************************************************/

//                                          POINTS

/**********************************************************************************************************************************/

/**
 * @brief MainWindow::initializePoints
 * initialize the points on the map and in the model
 */
void MainWindow::initializePoints(){
    //qDebug() << "initializePoints called";
    /// retrieves the points from the xml file and stores them in the model
    XMLParser pParser(":/xml/points.xml");
    pParser.readPoints(points, mapPixmapItem, this);

    points->addTmpPoint(mapPixmapItem, this);
    //qDebug() << "Nb points after init :" << points->count();
    mapPixmapItem->setPoints(points);
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

    std::shared_ptr<PointView> displaySelectedPointView = points->getTmpPointView();

    /// sets the pixmaps of the other points
    points->setPixmapAll(PointView::NORMAL);

    /// tmp point is blue
    displaySelectedPointView->setPixmap(PointView::MID);

    /// we are not modifying an existing point
    if(!leftMenu->getDisplaySelectedPoint()->getActionButtons()->getEditButton()->isChecked()){
    //qDebug() << "editing";
    leftMenu->show();

        displaySelectedPointView->setState(GraphicItemState::EDITING_PERM);
        hideAllWidgets();
        createPointWidget->setSelectedPoint(displaySelectedPointView);
        createPointWidget->show();
        float x = displaySelectedPointView->getPoint()->getPosition().getX();
        float y = displaySelectedPointView->getPoint()->getPosition().getY();

        if(map->getMapImage().pixelColor(x ,y).red() >= 254){
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
    } else {
        /// on the left we display the position of the temporary point as the user moves it around but we don't make any modifications on the model yet
        leftMenu->getDisplaySelectedPoint()->getXLabel()->setText(QString::number(points->getTmpPointView()->getPoint()->getPosition().getX()));
        leftMenu->getDisplaySelectedPoint()->getYLabel()->setText(QString::number(points->getTmpPointView()->getPoint()->getPosition().getY()));
        if(displaySelectedPointView->getPoint()->isHome()){
            leftMenu->getDisplaySelectedPoint()->getHomeWidget()->show();
        } else {
            leftMenu->getDisplaySelectedPoint()->getHomeWidget()->hide();
        }
    }
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
    topLayout->setEnabled(false);

    pointsLeftWidget->getGroupNameEdit()->setFocus();
    pointsLeftWidget->setCreatingGroup(true);
    pointsLeftWidget->getGroupButtonGroup()->uncheck();
    /// resets the name edit field
    pointsLeftWidget->getGroupNameEdit()->setText("");
    /// stops a user from creating a new group with no name
    pointsLeftWidget->getSaveButton()->setEnabled(false);
    /// uncheck and disable the buttons
    pointsLeftWidget->getActionButtons()->uncheckAll();

    pointsLeftWidget->getActionButtons()->enableAll();

    pointsLeftWidget->getActionButtons()->getPlusButton()->setToolTip("Enter a name for your group and click \"save\" or click \"cancel\" to cancel");

    /// to prevent the user from clicking on the buttons
    pointsLeftWidget->getGroupButtonGroup()->setEnabled(false);
    leftMenu->getReturnButton()->setEnabled(false);
    leftMenu->getCloseButton()->setEnabled(false);

    /// here we allow a user to create a new group
    pointsLeftWidget->getGroupNameEdit()->show();
    pointsLeftWidget->getGroupNameLabel()->show();
    pointsLeftWidget->getCancelButton()->show();
    pointsLeftWidget->getSaveButton()->show();
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

    leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->setGroup(leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getGroupIndex());
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
    leftMenu->getReturnButton()->setEnabled(false);
    leftMenu->getReturnButton()->setToolTip("Please save or discard your modifications before navigating the menu again.");

    leftMenu->getCloseButton()->setEnabled(false);

    /// update buttons enable attribute and tool tips
    leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMapButton()->setChecked(true);
    leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMinusButton()->setEnabled(false);
    leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMinusButton()->setToolTip("");
    leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMapButton()->setEnabled(false);
    leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMapButton()->setToolTip("");

    /// hide the temporary point on the map
    std::shared_ptr<PointView> displaySelectedPointView = points->findPointView(leftMenu->getDisplaySelectedPoint()->getPointName());
    qDebug() << "MainWindow::editPointButtonEvent selected point to edit " << leftMenu->getDisplaySelectedPoint()->getPointName();
    displaySelectedPointView->setOriginalPosition(displaySelectedPointView->getPoint()->getPosition());

    QMapIterator<QString, std::shared_ptr<QVector<std::shared_ptr<PointView>>>> i(*points->getGroups());
    while (i.hasNext()) {
        i.next();
        for(int j = 0; j < i.value()->size(); j++)
            qDebug() << i.key() << i.value()->at(j)->getPoint()->getName();
    }

    if(displaySelectedPointView && !(*(displaySelectedPointView->getPoint()) == *(points->getTmpPointView()->getPoint())))
        points->displayTmpPoint(false);

    /// change the color of the pointview that's selected on the map
    displaySelectedPointView->setPixmap(PointView::PixmapType::HOVER);
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

    /// makes it obvious what the user has to do to change the name of his point
    leftMenu->getDisplaySelectedPoint()->getNameEdit()->setAutoFillBackground(false);
    leftMenu->getDisplaySelectedPoint()->getNameEdit()->setFrame(true);

    /// if the point is a home point any modification of its name is forbidden
    /// not anymore since its name is no longer imposed by the application
    //if(!displaySelectedPointView->getPoint()->isHome())
    leftMenu->getDisplaySelectedPoint()->getNameEdit()->setReadOnly(false);

    /// sets the state of the map and the other widgets to prevent other concurrent actions
    setGraphicItemsState(GraphicItemState::NO_EVENT, false);
    mapPixmapItem->setState(GraphicItemState::EDITING_PERM);

    /// sets the state of the point of the map to make it draggable
    displaySelectedPointView->setState(GraphicItemState::EDITING_PERM);
    displaySelectedPointView->setFlag(QGraphicsItem::ItemIsMovable, true);
}

/**
 * @brief MainWindow::editGroupBtnEvent
 * @param checked
 * called when the user wants to edit a point from the first points menu
 */
void MainWindow::editGroupBtnEvent(){
    qDebug() << "editPointBtnEvent called" << pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton()->text();

    setEnableAll(false);
    int btnIndex = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedId();
    qDebug() << "btnIndex" << btnIndex;
    pointsLeftWidget->setLastCheckedId(pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton()->text());

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
        std::shared_ptr<PointView> pointView = points->findPointView(checkedId);

        /// must display the tick icon in the pointsLeftWidget
        pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton()->setIcon(QIcon(":/icons/eye_point.png"));
        if(pointView){
            pointView->show();
            QString robotName = "";
            if(pointView->getPoint()->isHome()){
                RobotView* rv = robots->findRobotUsingHome(pointView->getPoint()->getName());
                if(rv != NULL)
                    robotName = rv->getRobot()->getName();
                else
                    qDebug() << "editGroupBtnEvent : something unexpected happened";
            }
            leftMenu->getDisplaySelectedPoint()->setPointView(pointView.get(), robotName);
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
        leftMenu->getReturnButton()->setEnabled(false);
        leftMenu->getCloseButton()->setEnabled(false);

        pointsLeftWidget->getActionButtons()->getEditButton()->setToolTip("Type a new name for your group and press ENTER");
        /// disables the plus button
        pointsLeftWidget->getActionButtons()->getPlusButton()->setEnabled(false);
        /// disables the other buttons
        pointsLeftWidget->disableButtons();
        pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->setText(checkedId);

        pointsLeftWidget->getGroupButtonGroup()->uncheck();
        pointsLeftWidget->getGroupButtonGroup()->setEnabled(false);
        pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->selectAll();
        pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->setFocus();
        pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->show();


        pointsLeftWidget->getGroupButtonGroup()->getLayout()->removeWidget(pointsLeftWidget->getGroupButtonGroup()->getModifyEdit());
        pointsLeftWidget->getGroupButtonGroup()->getLayout()->insertWidget(btnIndex, pointsLeftWidget->getGroupButtonGroup()->getModifyEdit());
        pointsLeftWidget->getGroupButtonGroup()->setEditedGroupName(checkedId);
        btn->hide();
    }
}

void MainWindow::selectPointBtnEvent(){
    qDebug() << "selectPointBtnEvent called";
}

void MainWindow::switchFocus(QString name, QWidget* widget, MainWindow::WidgetType type)
{
    lastWidgets.append(QPair<QPair<QWidget*, QString>, MainWindow::WidgetType>(QPair<QWidget*, QString>(widget,name), type));

    if(lastWidgets.size()>1) {
        leftMenu->showBackButton(lastWidgets.at(lastWidgets.size()-2).first.second);
    } else {
        leftMenu->hideBackButton();
    }

    qDebug() << "__________________";

    for(int i=0;i<lastWidgets.size();i++) {
        qDebug() << lastWidgets.at(i).first.second;
    }

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
        std::shared_ptr<PointView> displaySelectedPointView = points->findPointView(leftMenu->getDisplaySelectedPoint()->getPointName());
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
        /// we reset the origin of the point information menu in order to display the buttons to go back in the further menus
        leftMenu->getDisplaySelectedPoint()->setOrigin(DisplaySelectedPoint::POINTS_MENU);
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
    createPointWidget->hideGroupLayout();

    points->addPoint(groupName, name, x, y, true, Point::PointType::TEMP, mapPixmapItem, this);

    /// saves it to the file
    XMLParser parser(XML_PATH);
    parser.save(*points);

    /// updates the menu
    pointsLeftWidget->updateGroupButtonGroup();

    /// hides the temporary point so that they don't superimpose which is confusing when hiding / showing the newly created point
    points->getTmpPointView()->hide();

    /// hide the creation widget
    hideAllWidgets();
    leftMenu->hide();

    setMessageTop(TEXT_COLOR_SUCCESS, "You have created a new point");
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
        break;
        case QMessageBox::Ok : {
            /// we first check that our point is not the home of a robot
            std::shared_ptr<Point> point = points->findPoint(pointName);
            if(!point->isHome()){
                qDebug() << "Go ahead and remove me I am not a home point anyway";

                /// updates the model
                points->removePoint(pointName);

                /// save changes in the file
                XMLParser parserPoints(XML_PATH);
                parserPoints.save(*points);

                /// updates the menu
                pointsLeftWidget->getGroupButtonGroup()->updateButtons();

                /// need to remove the point from the map
                pointsLeftWidget->setLastCheckedId("");
            } else {
                /// this is in fact the home point of a robot, we prompt a customized message to the end user
                RobotView* robot = robots->findRobotUsingHome(pointName);
                if(robot != NULL){
                    openInterdictionOfPointRemovalMessage(pointName, robot->getRobot()->getName());
                    qDebug() << "Sorry this point is the home of a robot and therefore cannot be removed";
                } else {
                    qDebug() << "setSelectedRobotFromPoint : something unexpected happened";
                }
            }
        }
        pointsLeftWidget->disableButtons();
        break;
        default:
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

                std::shared_ptr<PointView> point = points->findPointView(pointName);
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
                    XMLParser parserPoints(XML_PATH);
                    parserPoints.save(*points);

                    /// makes the buttons checkable again
                    leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->setCheckable(true);

                    /// prompts the user to ask him if he wants to delete the group in case it would be empty
                    if(points->getGroups()->value(group)->size() <= 0){
                        int res = openEmptyGroupMessage(group);
                        if(res == QMessageBox::Yes){
                            /// updates model
                            points->removeGroup(pointsLeftWidget->getLastCheckedId());

                            /// updates file
                            XMLParser parser(XML_PATH);
                            parser.save(*points);

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
                    RobotView* robot = robots->findRobotUsingHome(pointName);
                    if(robot != NULL){
                        qDebug() << robot->getRobot()->getName();
                        openInterdictionOfPointRemovalMessage(pointName, robot->getRobot()->getName());
                        qDebug() << "Sorry this point is the home of a robot and therefore cannot be removed";
                    } else {
                        qDebug() << "askForDeletePointConfirmation : something unexpected happened";
                    }
                }
            }
            leftMenu->getDisplaySelectedGroup()->disableButtons();
        break;
        default:
        /// should never be here
            qDebug() << "MainWindow::askForDeletePointConfirmation should not be here";
        break;
    }
}

/**
 * @brief MainWindow::askForDeleteGroupConfirmation
 * @param index
 * Called when a user wants to remove a whole group of points
 * the index given is the index of the group within the Points object
 */
void MainWindow::askForDeleteGroupConfirmation(QString index){
    qDebug() << "askForDeleteGroupConfirmation called";
    int ret = openConfirmMessage("Do you really want to remove this group ? All the points in this group would also be removed.");
    switch(ret){
        case QMessageBox::Cancel :
            qDebug() << "clicked no";
            pointsLeftWidget->getActionButtons()->getMinusButton()->setChecked(false);
        break;
        case QMessageBox::Ok : {
            /// we have to check that none of the points is the home of a robot
            QVector<QString> homePointNames = points->getHomeNameFromGroup(index);
            if(homePointNames.size() <= 0){

                /// removes all the points of the group on the map
                for(int i = 0; i < points->getGroups()->value(index)->size(); i++){
                    points->getGroups()->value(index)->at(i)->hide();
                }

                /// removes the group from the model
                points->removeGroup(index);

                /// updates the file
                XMLParser parserPoints(XML_PATH);
                parserPoints.save(*points);

                /// updates the menu
                pointsLeftWidget->getGroupButtonGroup()->updateButtons();
                pointsLeftWidget->getActionButtons()->getMinusButton()->setChecked(false);
                pointsLeftWidget->setLastCheckedId("");

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
                msgBox.setInformativeText("To modify the home point of a robot you can either click on the menu > Robots, choose a robot and Add home or simply click a robot on the map and Add home");
                msgBox.exec();
                qDebug() << "Sorry this point is the home of a robot and therefore cannot be removed";
            }
        }
        pointsLeftWidget->disableButtons();
        break;
        default:
            /// should never be here
            qDebug() << " dafuk ?";
        break;
    }
}

void MainWindow::displayPointEvent(PointView* pointView){
    qDebug() << "MainWindow::displayPointEvent called" << pointView->getPoint()->getName();


    /// If the point is not a path or is a path but from a permanent point, we display the menu with informations on the point
    if(!(pointView->getPoint()->isPath() && pointView->getPoint()->getName().contains(PATH_POINT_NAME))){

        /// resets the color of the previous selected point if such point exists
        if(pointView && !(*(pointView->getPoint()) == *(points->getTmpPointView()->getPoint())))
            points->displayTmpPoint(false);

        leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMapButton()->setChecked(true);

        QString robotName = "";
        if(pointView->getPoint()->isHome()){
            RobotView* rv = robots->findRobotUsingHome(pointView->getPoint()->getName());
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
            RobotView* rv = robots->findRobotUsingHome(pointView->getPoint()->getName());
            if(rv != NULL)
                robotName = rv->getRobot()->getName();
            else
                qDebug() << "MainWindow::displayPointEvent  : something unexpected happened";
        }
        pointView->setPixmap(PointView::PixmapType::MID);
        pointView->setLastPixmap(QPixmap(PIXMAP_MID));

        if(pointView->getPoint()->isPath()){
            /// if it's a path point the edition/suppression is forbidden from here
            leftMenu->getDisplaySelectedPoint()->getActionButtons()->getEditButton()->setEnabled(false);
            leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMinusButton()->setEnabled(false);
        }

    } else {
        /// The point is a path' point from a temporary point so we display the page of the robot in which this pathpoint is used
        RobotView* robot = robots->findRobotUsingTmpPointInPath(pointView->getPoint());
        if(robot){
            qDebug() << "MainWindow::displayPointEvent  At least, I found the robot" << robot->getRobot()->getName();
            resetFocus();
            setSelectedRobot(robot);
        }
    }
}

void MainWindow::displayGroupMapEvent(void){
    qDebug() << "MainWindow::displayGroupMapEvent called";

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
                    std::shared_ptr<PointView> point = points->getGroups()->value(checkedName)->at(i);
                    point->hide();

                }
                /// update the file
                XMLParser parserPoints(XML_PATH);
                parserPoints.save(*points);
            } else if(points->getGroups()->value(checkedName)->size() == 0) {
                pointsLeftWidget->getActionButtons()->getMapButton()->setChecked(false);
                topLayout->setLabelDelay(TEXT_COLOR_WARNING, "This group is empty. There is no points to display", 2000);
            } else {
                /// updates the tooltip of the map button
                pointsLeftWidget->getActionButtons()->getMapButton()->setToolTip("Click here to hide the selected group on the map");
                /// the group must now be displayed
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/folder_eye.png"));

                for(int i = 0; i < points->getGroups()->value(checkedName)->size(); i++){
                    std::shared_ptr<PointView> point = points->getGroups()->value(checkedName)->at(i);
                    point->show();
                    point->setPixmap(PointView::PixmapType::MID);
                    /// update the file
                    XMLParser parserPoints(XML_PATH);
                    parserPoints.save(*points);
                }
            }
        }
    }
    /// we display isolated points
    else if(points->isAPoint(checkedName)){
        std::shared_ptr<PointView> point = points->findPointView(checkedName);

        /// if the point is displayed we hide it
        if(point && point->isVisible()){
            /// updates the tooltip of the map button
            pointsLeftWidget->getActionButtons()->getMapButton()->setToolTip("Click here to display the selected point on the map");
            point->hide();

            /// update the file
            XMLParser parserPoints(XML_PATH);
            parserPoints.save(*points);

            /// we remove the tick icon
            pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/space_point.png"));
        } else {
            /// updates the tooltip of the map button
            pointsLeftWidget->getActionButtons()->getMapButton()->setToolTip("Click here to hide the selected point on the map");
            point->show();

            /// update the file
            XMLParser parserPoints(XML_PATH);
            parserPoints.save(*points);

            /// we add the tick icon
            pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/eye_point.png"));
        }
    }
}

void MainWindow::displayPointMapEvent(){
    qDebug() << "MainWindow::displayPointMapEvent called";
    std::shared_ptr<PointView> pointView = points->findPointView(leftMenu->getDisplaySelectedPoint()->getPointName());
    std::pair<QString, int> pointIndexes = points->findPointIndexes(pointView->getPoint()->getName());
    qDebug() << "Indexes are " << pointIndexes.first << pointIndexes.second;
    int checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonIdByName(pointIndexes.first);

    if(pointView && pointView->getPoint()){
        if(pointView->isVisible()){
            qDebug() << "MainWindow::displayPointMapEvent hiding" << pointView->getPoint()->getName();
            leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMapButton()->setToolTip("Click to display this point");
            pointView->hide();

            /// update the file
            XMLParser parserPoints(XML_PATH);
            parserPoints.save(*points);

            /// we update the group menu
            leftMenu->updateGroupDisplayed(points, pointIndexes.first);

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
            leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMapButton()->setToolTip("Click to hide this point");
            pointView->show();

            /// update the file
            XMLParser parserPoints(XML_PATH);
            parserPoints.save(*points);

            /// we update the groups menu
            /// it's a point that belongs to a group
            if(pointIndexes.first.compare(NO_GROUP_NAME) != 0){
                qDebug() << pointIndexes.second;
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/eye_point.png"));
                //leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->)->setIcon(QIcon(":/icons/eye_point.png"));
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
       leftMenu->updateGroupDisplayed(points, checkedName);
       selectedGroup->getPointButtonGroup()->setCheckable(true);
       selectedGroup->show();
       selectedGroup->setName(checkedName);

       switchFocus(checkedName, selectedGroup, MainWindow::WidgetType::GROUP);
       setMessageTop(TEXT_COLOR_INFO, "CLick the map to add a permanent point");

    } else if(points->isAPoint(checkedName)){
        /// it's an isolated point
        DisplaySelectedPoint* selectedPoint = leftMenu->getDisplaySelectedPoint();
        std::shared_ptr<PointView> pointView = points->findPointView(checkedName);
        QString robotName = "";

        if(pointView && pointView->getPoint()->isHome()){
            RobotView* rv = robots->findRobotUsingHome(checkedName);
            if(rv != NULL)
                robotName = rv->getRobot()->getName();
            else
                qDebug() << "MainWindow::displayPointsInGroup something unexpected happened";
        }

        selectedPoint->setPointView(pointView.get(), robotName);
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

/// TODO voir si a supprimer
/*void MainWindow::removePoint(std::shared_ptr<Point>& point, const Origin origin){
    qDebug() << "removepoint event called";
    int answer = openConfirmMessage("Do you really want to remove this point");
    switch(answer){
    case QMessageBox::No:
        if(origin == Origin::POINTS)
            leftMenu->disableButtons();
        break;
    case QMessageBox::Ok:
        if(!point->isHome()){
            std::pair<int, int> pointIndexes = points->findPointIndexes(point->getName());
            std::shared_ptr<Group> group = points->getGroups().at(pointIndexes.first);
            qDebug() << "Go ahead and remove me I am not a home point anyway";
            /// need to remove the point from the map
            pointViews->getPointViewFromPoint(*point)->hide();
            /// updates the model
            group->removePoint(pointIndexes.second);
            if(origin == Origin::POINTS){
                /// updates the group
                leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->setGroup(points, pointIndexes.first);
                /// makes the buttons checkable again
                leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->setCheckable(true);
            }
            if(origin == Origin::GROUPS)
                pointsLeftWidget->disableButtons();

            /// save the changes to the file
            XMLParser parserPoints(XML_PATH, mapPixmapItem);
            parserPoints.save(*points);
            /// prompts the user to ask him if he wants to delete the group in case it would be empty and not the default group
            if(pointIndexes.first != points->count()-1 && group->isEmpty()){
                int confirmation = openEmptyGroupMessage(group->getName());
                if(confirmation == QMessageBox::Yes){
                    /// updates model
                    points->removeGroup(pointIndexes.first);
                    /// updates file
                    XMLParser parser(XML_PATH, mapPixmapItem);
                    parser.save(*points);
                    /// updates menu
                    pointsLeftWidget->getGroupButtonGroup()->update(*points);
                    /// hides group menu and shows list of groups menu (if we come from there)
                    leftMenu->getDisplaySelectedGroup()->hide();
                    if(origin == Origin::GROUPS)
                        pointsLeftWidget->show();
                    /// updates the list of available groups when a user creates a point
                    createPointWidget->updateGroupBox(*points);
                    backEvent();
                }
            }
        } else {
            /// this is in fact the home point of a robot, we prompt a customized message to the end user
            RobotView* robot = robots->findRobotUsingHome(point->getName());
            if(robot != NULL){
                openInterdictionOfPointRemovalMessage(point->getName(), robot->getRobot()->getName());
                qDebug() << "Sorry this point is the home of a robot and therefore cannot be removed";
            } else {
                qDebug() << "removePoint : something unexpected happened";
            }
        }
        break;
    default:
        break;
    }
}*/

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
            std::shared_ptr<PointView> pointView = points->findPointView(leftMenu->getDisplaySelectedPoint()->getPointName());
            if(pointView && !pointView->getPoint()->isHome()){
                QString pointName = pointView->getPoint()->getName();

                /// holds the index of the group and the index of a particular point in this group within <points>
                std::pair<QString, int> pointIndexes = points->findPointIndexes(pointName);
                if(pointIndexes.first.compare("")!= 0){
                    qDebug() << "groupindex " << pointIndexes.first;
                    /// it's an isolated point
                    if(points->isAPoint(pointIndexes.first)){
                        /// need to remove the point from the map
                        pointView->hide();

                        /// updates the model
                        points->removePoint(pointName);

                        /// updates the file containing containing points info
                        XMLParser parserPoints(XML_PATH);
                        parserPoints.save(*points);

                        /// updates the list of points
                        pointsLeftWidget->getGroupButtonGroup()->updateButtons();
                        backEvent();


                    } else {
                        /// need to remove the point from the map
                        pointView->hide();

                        /// updates the model
                        points->removePoint(pointName);

                        /// updates the file containing containing points info
                        XMLParser parserPoints(XML_PATH);
                        parserPoints.save(*points);

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
                                XMLParser parser(XML_PATH);
                                parser.save(*points);

                                /// updates menu
                                pointsLeftWidget->getGroupButtonGroup()->updateButtons();
                                createPointWidget->updateGroupBox();
                                backEvent();
                            }
                        }
                        updateAllPaths();
                        setTemporaryMessageTop(TEXT_COLOR_SUCCESS, "You have deleted the point : " + pointName + " from the group : " + pointIndexes.first, 2500);
                    }
                } else {
                    qDebug() << "could not find this point";
                }
            } else {
                /// this point is actually the home point of a robot and therefore cannot be removed
                RobotView* robot = robots->findRobotUsingHome(pointView->getPoint()->getName());
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
    leftMenu->getCloseButton()->setEnabled(false);
    leftMenu->getReturnButton()->setEnabled(false);
    setMessageTop(TEXT_COLOR_INFO, "Click the map or drag the point to change its position");
    QString groupName = leftMenu->getDisplaySelectedGroup()->getNameLabel()->text();

    qDebug() << "working on group" << groupName << "and id" << leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->checkedId();

    QString pointName = leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->checkedButton()->text();

    if(pointName.compare("") != 0){
        /// update the pointview and show the point on the map with hover color
        QString robotName = "";
        if(points->findPointView(pointName)->getPoint()->isHome()){
            RobotView* rv = robots->findRobotUsingHome(pointName);
            if(rv != NULL)
                robotName = rv->getRobot()->getName();
            else
                qDebug() << "editPointFromGroupMenu : something unexpected happened";
        }
        qDebug() << "name point u trying to edit" << pointName;
        std::shared_ptr<PointView> displaySelectedPointView = points->findPointView(pointName);
        displaySelectedPointView->setOriginalPosition(displaySelectedPointView->getPoint()->getPosition());

        if(displaySelectedPointView){
            qDebug() << "about to put u orange";

            leftMenu->getDisplaySelectedPoint()->setPointView(displaySelectedPointView.get(), robotName);

            displaySelectedPointView->setPixmap(PointView::PixmapType::HOVER);
            displaySelectedPointView->show();

            /// update the file
            XMLParser parser(XML_PATH);
            parser.save(*points);

            leftMenu->getDisplaySelectedPoint()->getNameEdit()->setAutoFillBackground(false);
            leftMenu->getDisplaySelectedPoint()->getNameEdit()->setFrame(true);

            /// if the point is a home point any modification of its name is forbidden

            if(!displaySelectedPointView->getPoint()->isHome())
                leftMenu->getDisplaySelectedPoint()->getNameEdit()->setReadOnly(false);

            /// sets the state of the map and the other widgets to prevent other concurrent actions
            setGraphicItemsState(GraphicItemState::NO_EVENT, false);
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
            leftMenu->getDisplaySelectedPoint()->getNameEdit()->setReadOnly(false);
            leftMenu->getDisplaySelectedPoint()->getCancelButton()->show();
            leftMenu->getDisplaySelectedPoint()->getSaveButton()->show();
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
    QString pointName = leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->checkedButton()->text();
    std::shared_ptr<PointView> pointView = points->findPointView(pointName);

    if(pointName.compare("") != 0 && pointView){
        setMessageTop(TEXT_COLOR_NORMAL, "");
        DisplaySelectedPoint* selectedPoint = leftMenu->getDisplaySelectedPoint();

        QString robotName = "";
        if(pointView->getPoint()->isHome()){
            RobotView* rv = robots->findRobotUsingHome(pointName);
            if(rv != NULL)
                robotName = rv->getRobot()->getName();
            else
                qDebug() << "setSelectedRobotFromPoint : something unexpected happened";
        }

        selectedPoint->setPointView(pointView.get(), robotName);
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
    qDebug() << "MainWindow::updatePoint called";
    leftMenu->getCloseButton()->setEnabled(true);
    topLayout->setEnabled(true);
    ///resets the tooltip of the edit button and the minus button
    leftMenu->getDisplaySelectedPoint()->getActionButtons()->getEditButton()->setToolTip("You can click on this button and then choose between clicking on the map or drag the point to change its position");
    leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMinusButton()->setToolTip("You can click this button to remove the point");

    leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMapButton()->setEnabled(true);
    leftMenu->getDisplaySelectedPoint()->getActionButtons()->getMapButton()->setToolTip("Click to hide this point");

    DisplaySelectedPoint* selectedPoint = leftMenu->getDisplaySelectedPoint();
    qDebug() << "MainWindow::updatePoint updating point" << selectedPoint->getPointName();
    std::shared_ptr<PointView> displaySelectedPointView = points->findPointView(selectedPoint->getPointName());
    //selectedPoint->setPointName(leftMenu->getDisplaySelectedPoint()->getNameEdit()->text());

    /// resets the color of the pointView
    if(displaySelectedPointView){
        displaySelectedPointView->setPixmap(PointView::PixmapType::NORMAL);

        /// notifies the map that the point's name has changed and that the hover has to be updated
        emit nameChanged(displaySelectedPointView->getPoint()->getName(), leftMenu->getDisplaySelectedPoint()->getNameEdit()->text());

        /// updates the name in the label
        displaySelectedPointView->getPoint()->setName(leftMenu->getDisplaySelectedPoint()->getNameEdit()->text());

        /// updates the position of the point
        /// to determine wheter the coordinate is 2 digits long or 3 digits long in order to parse them correctly
        int xLength = leftMenu->getDisplaySelectedPoint()->getXLabel()->text().count();
        int yLength = leftMenu->getDisplaySelectedPoint()->getYLabel()->text().count();

        displaySelectedPointView->setPos(
        leftMenu->getDisplaySelectedPoint()->getXLabel()->text().right(xLength-4).toFloat(),
        leftMenu->getDisplaySelectedPoint()->getYLabel()->text().right(yLength-4).toFloat());
    }

    /// save changes to the file
    XMLParser parserPoints(XML_PATH);
    parserPoints.save(*points);

    /// to change the aspect of the point name
    leftMenu->getDisplaySelectedPoint()->getNameEdit()->setAutoFillBackground(true);
    leftMenu->getDisplaySelectedPoint()->getNameEdit()->setFrame(false);

    /// so that the name cannot be changed anymore unless you click the edit button again
    selectedPoint->getNameEdit()->setReadOnly(true);

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
    updateAllPaths();

    setMessageTop(TEXT_COLOR_SUCCESS, "Your point has been modified");
    delay(1500);
    setMessageTop(TEXT_COLOR_NORMAL, "");
    topLayout->setLabelDelay(TEXT_COLOR_SUCCESS, "Your point has been modified", 1500);

}

/**
 * @brief MainWindow::cancelEvent
 * called when a user discard the changes made about a point
 */
void MainWindow::cancelEvent(void){
    qDebug() << "cancel edit point event called";
    leftMenu->getCloseButton()->setEnabled(true);
    leftMenu->getReturnButton()->setEnabled(true);
    topLayout->setEnabled(true);
    /// reset the color of the pointView
    std::shared_ptr<PointView> displaySelectedPointView = points->findPointView(leftMenu->getDisplaySelectedPoint()->getPointName());
    if(displaySelectedPointView){
        displaySelectedPointView->setPixmap(PointView::PixmapType::NORMAL);
        qDebug() << "about to reset your position";

        displaySelectedPointView->getPoint()->setPosition(displaySelectedPointView->getOriginalPosition());

        /// to change the aspect of the point name
        leftMenu->getDisplaySelectedPoint()->getNameEdit()->setAutoFillBackground(true);
        leftMenu->getDisplaySelectedPoint()->getNameEdit()->setFrame(false);

        /// we hide the buttons relative to the edit option and make sure the points properties are not longer modifiable
        leftMenu->getDisplaySelectedPoint()->getNameEdit()->setReadOnly(true);
        leftMenu->getDisplaySelectedPoint()->getActionButtons()->getEditButton()->setChecked(false);
        leftMenu->getDisplaySelectedPoint()->getCancelButton()->hide();
        leftMenu->getDisplaySelectedPoint()->getSaveButton()->hide();

        /// in case the user had dragged the point around the map or clicked it, this resets the coordinates displayed to the original ones
        leftMenu->getDisplaySelectedPoint()->getXLabel()->setText(QString::number(
                                                                      displaySelectedPointView->getPoint()->getPosition().getX()));
        leftMenu->getDisplaySelectedPoint()->getYLabel()->setText(QString::number(
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


    topLayout->setLabelDelay(TEXT_COLOR_WARNING, "Your point has not been modified",1500);

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
    QString checkedId = leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->checkedButton()->text();

    if(checkedId.compare("") != 0)
        askForDeletePointConfirmation(checkedId);
    else
        qDebug() << "can't remove point without name";
    leftMenu->getDisplaySelectedGroup()->getActionButtons()->getMinusButton()->setChecked(false);
}

/**
 * @brief MainWindow::displayPointFromGroupMenu
 * called when a user displays or hides a point on the map from the group menu
 */
void MainWindow::displayPointFromGroupMenu(){

    qDebug() << "1" << (leftMenu->getDisplaySelectedGroup() == NULL);
    qDebug() << "2" << (leftMenu->getDisplaySelectedGroup()->getPointButtonGroup() == NULL);
    qDebug() << "3" << (leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup() == NULL);
    qDebug() << "4" << (leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->checkedButton() == NULL);
    const QString pointName = leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->checkedButton()->text();
    qDebug() << "displaypointfromgroupmenu event called" << pointName ;

    int checkedId = leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonIdByName(pointName);


    //if(pointName.compare("") == 0){
    if(checkedId != -1){
        std::shared_ptr<PointView> currentPointView = points->findPointView(pointName);

        /// if the point is displayed we stop displaying it
        if(currentPointView->isVisible()){

            /// hides the point on the map
            currentPointView->hide();

            /// removes the tick icon to show that the point is not displayed on the map
            leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->buttons()[checkedId]->setIcon(QIcon(":/icons/space_point.png"));

            /// updates the file
            XMLParser parserPoints(XML_PATH);
            parserPoints.save(*points);

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
            XMLParser parserPoints(XML_PATH);
            parserPoints.save(*points);

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
    msgBox.setInformativeText("To modify the home point of a robot you can either click on the menu > Robots, choose a robot and Add home or simply click a robot on the map and Add home");
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
    std::shared_ptr<PointView> pointView = points->findPointView(pointName);

    if(pointView){
        DisplaySelectedPoint* selectedPoint = leftMenu->getDisplaySelectedPoint();
        QString robotName = "";
        if(pointView->getPoint()->isHome()){
            RobotView* rv = robots->findRobotUsingHome(pointName);
            if(rv != NULL)
                robotName = rv->getRobot()->getName();
            else
                qDebug() << "doubleClickOnPoint : something unexpected happened";
        }
        selectedPoint->setPointView(pointView.get(), robotName);
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
       leftMenu->updateGroupDisplayed(points, checkedName);
       selectedGroup->getPointButtonGroup()->setCheckable(true);
       selectedGroup->show();
       selectedGroup->setName(checkedName);

       switchFocus(checkedName, selectedGroup, MainWindow::WidgetType::GROUP);
    } else if(points->isAPoint(checkedName)){

        /// it's an isolated point
        setMessageTop(TEXT_COLOR_NORMAL, "");
        DisplaySelectedPoint* selectedPoint = leftMenu->getDisplaySelectedPoint();
        std::shared_ptr<PointView> pointView = points->findPointView(checkedName);

        QString robotName = "";
        if(pointView->getPoint()->isHome()){
            RobotView* rv = robots->findRobotUsingHome(pointView->getPoint()->getName());
            if(rv != NULL)
                robotName = rv->getRobot()->getName();
            else
                qDebug() << "doubleClickOnGroup : something unexpected happened";
        }

        selectedPoint->setPointView(pointView.get(), robotName);
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
        XMLParser parser(XML_PATH);
        parser.save(*points);

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

        topLayout->setLabelDelay(TEXT_COLOR_SUCCESS, "You have created a new group",2500);
    } else if(pointsLeftWidget->checkGroupName(groupName) == 1){
        topLayout->setLabelDelay(TEXT_COLOR_DANGER, "The name of your group cannot be empty",2500);
    } else {
        topLayout->setLabelDelay(TEXT_COLOR_DANGER, "You cannot choose : " + groupName + " as a new name for your group because another group already has this name",2500);
    }
}

void MainWindow::modifyGroupWithEnter(QString name){
    name = name.simplified();
    qDebug() << "modifying group after enter key pressed from" << pointsLeftWidget->getLastCheckedId() << "to" << name;

    topLayout->setEnabled(true);

    if(pointsLeftWidget->checkGroupName(name) == 0){
        leftMenu->getCloseButton()->setEnabled(true);

        /// Update the model
        points->getGroups()->insert(name, points->getGroups()->take(pointsLeftWidget->getLastCheckedId()));

        /// updates the group box to create a point
        createPointWidget->updateGroupBox();

        /// enables the plus button
        pointsLeftWidget->getActionButtons()->getPlusButton()->setEnabled(true);

        /// saves to file
        XMLParser parser(XML_PATH);
        parser.save(*points);

        /// enables the buttons
        pointsLeftWidget->getGroupButtonGroup()->setEnabled(true);
        pointsLeftWidget->disableButtons();

        /// updates view
        int checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonIdByName(pointsLeftWidget->getGroupButtonGroup()->getEditedGroupName());
        pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->show();
        pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setText(name);


        pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->hide();

        leftMenu->getReturnButton()->setEnabled(true);
        pointsLeftWidget->setLastCheckedId("");
        topLayout->setLabelDelay(TEXT_COLOR_SUCCESS, "You have successfully modified the name of your group",1500);
    } else if(pointsLeftWidget->checkGroupName(name) == 1){
        topLayout->setLabelDelay(TEXT_COLOR_DANGER, "The name of your group cannot be empty. Please choose a name for your group",2500);
    } else {
        topLayout->setLabelDelay(TEXT_COLOR_DANGER, "You cannot choose : " + name + " as a new name for your group because another group already has this name",2500);
    }
}

void MainWindow::modifyGroupAfterClick(QString name){
    name = name.simplified();
    qDebug() << "modifyGroupAfterClick called from" << pointsLeftWidget->getLastCheckedId() << "to" << name;
    if (pointsLeftWidget->getLastCheckedId()!="")
     {
        topLayout->setEnabled(true);

        /// resets the menu
        leftMenu->getCloseButton()->setEnabled(true);

        pointsLeftWidget->getActionButtons()->getPlusButton()->setEnabled(true);
        pointsLeftWidget->getGroupButtonGroup()->setEnabled(true);
        pointsLeftWidget->disableButtons();

        pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->hide();
        leftMenu->getReturnButton()->setEnabled(true);
        int checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonIdByName(pointsLeftWidget->getGroupButtonGroup()->getEditedGroupName());
           QString color ="";
           QString msg = "";
           int time=1500;
        if(pointsLeftWidget->checkGroupName(name) == 0){
            /// Update the model
            qDebug() <<   pointsLeftWidget->getLastCheckedId();
            points->getGroups()->insert(name, points->getGroups()->take(pointsLeftWidget->getLastCheckedId()));

            /// saves to file
            XMLParser parser(XML_PATH);
            parser.save(*points);

            /// updates view
            pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setText(name);
            color= TEXT_COLOR_SUCCESS;
            msg= "You have successfully modified the name of your group";
            time=1500;
        } else if(pointsLeftWidget->checkGroupName(name) == 1){
            color=TEXT_COLOR_DANGER;
            msg= "The name of your group cannot be empty. Please choose a name for your group";
            time=2500;
        } else {
            color=TEXT_COLOR_DANGER;
            msg = "You cannot choose : " + name.simplified() + " as a new name for your group because another group already has this name";
            time = 2500;
        }

        pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->show();
         pointsLeftWidget->setLastCheckedId("");
         topLayout->setLabelDelay(color, msg,time);

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
        qDebug() << "Should never be here, if you do get here however, check that you have not added a new error code and forgotten to add it in the cases afterwards";
        break;
    }
}


/**********************************************************************************************************************************/

//                                          ODDS AND ENDS

/**********************************************************************************************************************************/

void MainWindow::quit(){
    close();
}

void MainWindow::backEvent(){
    //qDebug() << "back event called";

    //debug

    setEnableAll(true);
    /// resets the menus
    pointsLeftWidget->disableButtons();
    pointsLeftWidget->getGroupButtonGroup()->uncheck();
    leftMenu->getDisplaySelectedGroup()->disableButtons();
    leftMenu->getDisplaySelectedGroup()->uncheck();

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
void MainWindow::setGraphicItemsState(const GraphicItemState state, const bool clear){
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
    selectedRobotWidget->hide();
    robotsLeftWidget->hide();
    mapLeftWidget->hide();
    editSelectedRobotWidget->hide();
    createPointWidget->hide();
    leftMenu->getDisplaySelectedPoint()->hide();
    pathCreationWidget->hide();
    leftMenu->getDisplaySelectedGroup()->hide();
}

void MainWindow::clearNewMap(){
    qDebug() << "clearNewMap called";

    /// Clear all the paths and home
    /*for(int i = 0; i < robots->getRobotsVector().size(); i++){
        clearPath(i);
        robots->getRobotsVector().at(i)->getRobot()->setHome(NULL);
    }*/

    selectedPoint = NULL;
    editedPointView = NULL;

    /// Clear the list of points
    points->clear();

    /// Save the new list in the XML
    XMLParser parserPoints(XML_PATH);
    parserPoints.save(*points);

    /// Update the left menu displaying the list of groups and buttons
    pointsLeftWidget->updateGroupButtonGroup();
}

void MainWindow::delay(const int ms) const{
    QTime dieTime= QTime::currentTime().addMSecs(ms);
    while (QTime::currentTime() < dieTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

void MainWindow::setEnableAll(bool enable, GraphicItemState state, bool clearPath, int noReturn){
    setGraphicItemsState(state, clearPath);
    bottomLayout->setEnable(enable);
    topLayout->setEnable(enable);
    if(noReturn == -1)
        leftMenu->setEnableReturnCloseButtons(enable);
    else
        leftMenu->setEnableReturnCloseButtons(noReturn);
}

void MainWindow::centerMap(){
    scene->views().at(0)->centerOn(mapPixmapItem);
}

void MainWindow::settingBtnSlot(){
    qDebug() << "settingBtnSlot called";
}

void MainWindow::setTemporaryMessageTop(const QString type, const QString message, const int ms){
    setMessageTop(type, message);
    delay(ms);
    setMessageTop(TEXT_COLOR_NORMAL, "");
}
