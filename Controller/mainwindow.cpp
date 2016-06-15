#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "Controller/scanmetadatathread.h"
#include "Controller/scanrobotthread.h"
#include "Controller/scanmapthread.h"
#include "Model/pathpoint.h"
#include "Model/map.h"
#include "Model/robots.h"
#include "Model/robot.h"
#include "Model/group.h"
#include "Model/xmlparser.h"
#include "View/customqgraphicsview.h"
#include "View/mapview.h"
#include "View/leftmenu.h"
#include "View/pointview.h"
#include "View/leftmenuwidget.h"
#include "View/editselectedrobotwidget.h"
#include "View/selectedpointwidget.h"
#include "View/editselectedpointwidget.h"
#include "View/bottomlayout.h"
#include "View/pointsleftwidget.h"
#include "View/selectedrobotwidget.h"
#include "View/robotsleftwidget.h"
#include "View/mapleftwidget.h"
#include "View/pointsview.h"
#include "View/displayselectedpoint.h"
#include "View/displayselectedgroup.h"
#include "View/pathcreationwidget.h"
#include "View/groupbuttongroup.h"
#include "View/robotbtngroup.h"
#include "View/groupview.h"
#include "View/pathpainter.h"
#include <QVBoxLayout>
#include <QAbstractButton>
#include "View/pointbuttongroup.h"
#include "View/verticalscrollarea.h"

//TODO  stop threads/connections when scanning the map is finished/the user stop it

/**
 * @brief MainWindow::MainWindow
 * @param parent
 * The main controller of the application
 */
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    map = new Map();

    map->setMapFromFile(":/maps/map.pgm");
    robots = new Robots();
    scene = new QGraphicsScene(this);
    graphicsView = new CustomQGraphicsView(scene, this);
    selectedRobot = NULL;
    scanningRobot = NULL;
    selectedPoint = NULL;
    editedPointView = NULL;

    //create the toolbar
    initializeMenu();

    initializePoints();

    qDebug() << pointViews->getGroups().at(0).getPointViews().size();
    qDebug() << pointViews->getGroups().at(1).getPointViews().size();
    qDebug() << pointViews->getGroups().at(2).getPointViews().size();

    //create the graphic item of the map
    QPixmap pixmap = QPixmap::fromImage(map->getMapImage());
    mapPixmapItem = new MapView(pixmap, QSize(geometry().width(), geometry().height()), pointViews, this);
    connect(mapPixmapItem, SIGNAL(addPathPointMapView(Point*)), this, SLOT(addPathPoint(Point*)));

    pathPainter = new PathPainter(mapPixmapItem, pointViews);

    initializeRobots();

    scene->addItem(mapPixmapItem);

    // hide the scroll bars
    graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    graphicsView->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));

    leftMenu = new LeftMenu(this, points, robots, pointViews);
    initializeLeftMenu();
    ui->horizontalLayout->addWidget(leftMenu);


    rightLayout = new QVBoxLayout();
    ui->horizontalLayout->addLayout(rightLayout);
    rightLayout->addWidget(graphicsView);

    initializeBottomPanel();

    graphicsView->show();

    /// to update the names of the points displayed when a user changes the name of a point via the edit button
    connect(leftMenu->getDisplaySelectedPoint(), SIGNAL(nameChanged(QString, QString)), mapPixmapItem, SLOT(updateHover(QString, QString)));

}


MainWindow::~MainWindow(){
    delete ui;
    if(robotThread->isRunning()){
        robotThread->exit();
    }
    if(metadataThread->isRunning()){
        metadataThread->exit();
    }
    if(mapThread->isRunning()){
        mapThread->exit();
    }
    delete metadataThread;
    delete robotThread;
    delete mapThread;
    delete rightLayout;
    delete toolbar;
    delete graphicsView;
    delete map;
    delete robots;
    delete scene;
    delete mapPixmapItem;
    delete menuBar;
    delete addPointAction;
    delete selectedRobot;
    delete scanningRobot;
    delete pointViews;
    delete selectedPoint;
    delete leftMenu;
    delete bottomLayout;
    delete pathPainter;
}


void MainWindow::updateRobot(const float posX, const float posY, const float oriZ){
    qDebug() << "oki doki";

    float newPosX = (-map->getOrigin().getX()+posX)/map->getResolution()+ROBOT_WIDTH;
    float newPosY = map->getHeight()-(-map->getOrigin().getY()+posY)/map->getResolution()-ROBOT_WIDTH/2;

    float ori = asin(-oriZ) * 360.0 / PI + 90;
    scanningRobot->setPosition(newPosX, newPosY);
    scanningRobot->setOrientation(ori);
    qDebug() << "Robot position : " << scanningRobot->getRobot()->getPosition().getX()
             << " " << scanningRobot->getRobot()->getPosition().getY()
             << " " << scanningRobot->getRobot()->getOrientation();
    scene->update();
}


void MainWindow::updateMetadata(const int width, const int height, const float resolution,
                                const float originX, const float originY){
    map->setWidth(width);
    map->setHeight(height);
    map->setResolution(resolution);
    map->setOrigin(Position(originX, originY));
    qDebug() << "Map metadata : " << map->getWidth() << " " << map->getHeight() << " "
             << map->getResolution() << " " << map->getOrigin().getX()  << " " << map->getOrigin().getY() ;
}


void MainWindow::updateMap(const QByteArray mapArray){
    map->setMapFromArray(mapArray);
    QPixmap pixmap = QPixmap::fromImage(map->getMapImage());
    mapPixmapItem->setPixmap(pixmap);
    scene->update();
}


void MainWindow::connectToRobot(){
    qDebug() << "\n\nConnection";

    if(selectedRobot != NULL){
        QString ip = selectedRobot->getRobot()->getIp();
        qDebug() << "Trying to connect to : " << ip;
        metadataThread = new ScanMetadataThread(ip, PORT_MAP_METADATA);
        robotThread = new ScanRobotThread(ip, PORT_ROBOT_POS);
        mapThread = new ScanMapThread(ip, PORT_MAP);

        connect(robotThread, SIGNAL(valueChangedRobot(float, float, float))
                ,this ,SLOT(updateRobot(float, float, float)));

        connect(metadataThread, SIGNAL(valueChangedMetadata(int, int, float, float, float))
                ,this ,SLOT(updateMetadata(int, int, float, float, float)));

        connect(mapThread, SIGNAL(valueChangedMap(QByteArray))
                ,this ,SLOT(updateMap(QByteArray)));


        metadataThread->start();
        metadataThread->moveToThread(metadataThread);

        robotThread->start();
        robotThread->moveToThread(robotThread);

        mapThread->start();
        mapThread->moveToThread(mapThread);

        scanningRobot = selectedRobot;

    } else {
        qDebug() << "Select a robot first";
    }

}

void MainWindow::initializeMenu(){
    /// to create the toolbar
    QPixmap connectPix(":/icons/wifi.png");
    QPixmap quitPix(":/icons/close.png");
    QPixmap leftMenuPix(":/icons/list.png");

    toolbar = addToolBar("main");
    QAction *leftMenuAction = toolbar->addAction(QIcon(leftMenuPix),
        "Open menu");
    QAction *connectAction = toolbar->addAction(QIcon(connectPix),
        "Connect");

    // a separator for a esthetic purpose
    toolbar->addSeparator();

    QAction *quitAction = toolbar->addAction(QIcon(quitPix),
        "Quit Application");

    connect(quitAction, SIGNAL(triggered()), this, SLOT(quit()));
    connect(connectAction, SIGNAL(triggered()), this, SLOT(connectToRobot()));
    connect(leftMenuAction, SIGNAL(triggered()), this, SLOT(openLeftMenu()));

    toolbar->setIconSize(this->size()/10);

    ///create the menu
    menuBar = new QMenuBar();

    QMenu *file = new QMenu();
    file = menuBar->addMenu("&File");
    file->addAction(connectAction);
    file->addSeparator();
    file->addAction(quitAction);

    setMenuBar(menuBar);

    ///set some shortcut
    quitAction->setShortcut(tr("CTRL+Q"));
    connectAction->setShortcut(tr("CTRL+N"));

}

void MainWindow::initializeLeftMenu(){
    lastWidget = leftMenu->getLastWidget();
    leftMenuWidget = leftMenu->getLeftMenuWidget();
    pointsLeftWidget = leftMenu->getPointsLeftWidget();
    selectedRobotWidget = leftMenu->getSelectedRobotWidget();
    robotsLeftWidget = leftMenu->getRobotsLeftWidget();
    mapLeftWidget = leftMenu->getMapLeftWidget();
    editSelectedRobotWidget = leftMenu->getEditSelectedRobotWidget();
    selectedPointWidget = leftMenu->getSelectedPointWidget();
    editSelectedPointWidget = leftMenu->getEditSelectedPointWidget();
    pathCreationWidget = leftMenu->getPathCreationWidget();
}

void MainWindow::initializeRobots(){
    //TODO For dev, need to come from XML
    Robot* robot1 = new Robot("Roboty", "localhost", PORT_CMD, this);
    robot1->setWifi("Swaghetti Yolognaise");

    QVector<PathPoint*> path1;
    qDebug() << "Nb groups : " << points.getGroups().size();

    if(!points.getGroups().isEmpty()){
        qDebug() << "Nb points first group : " << points.getGroups().at(0)->getPoints().size();
        if(!points.getGroups().at(0)->getPoints().isEmpty()){
            if(points.getGroups().at(0)->getPoints().size() == 1){
                PathPoint* pathPoint1 = new PathPoint(*(points.getGroups().at(0)->getPoints().at(0)),
                                                      PathPoint::HUMAN_ACTION);
                path1.push_back(pathPoint1);
                path1.push_back(pathPoint1);
                path1.push_back(pathPoint1);
                path1.push_back(pathPoint1);
                path1.push_back(pathPoint1);
                path1.push_back(pathPoint1);
                path1.push_back(pathPoint1);
                path1.push_back(pathPoint1);
                path1.push_back(pathPoint1);
                path1.push_back(pathPoint1);
            }

            if(points.getGroups().at(0)->getPoints().size() == 2){

                PathPoint* pathPoint1 = new PathPoint(*(points.getGroups().at(0)->getPoints().at(0)),
                                                      PathPoint::HUMAN_ACTION);
                PathPoint* pathPoint2 = new PathPoint(*(points.getGroups().at(0)->getPoints().at(1)),
                                                      PathPoint::WAIT, 200);
                path1.push_back(pathPoint1);
                path1.push_back(pathPoint2);

                path1.push_back(pathPoint1);
                path1.push_back(pathPoint2);

                path1.push_back(pathPoint1);
                path1.push_back(pathPoint2);

                path1.push_back(pathPoint1);
                path1.push_back(pathPoint2);

                path1.push_back(pathPoint1);
                path1.push_back(pathPoint2);
            }

            if(points.getGroups().at(0)->getPoints().size() > 2){
                PathPoint* pathPoint1 = new PathPoint(*(points.getGroups().at(0)->getPoints().at(0)),
                                                      PathPoint::HUMAN_ACTION);
                PathPoint* pathPoint2 = new PathPoint(*(points.getGroups().at(0)->getPoints().at(1)),
                                                      PathPoint::WAIT, 200);
                PathPoint* pathPoint3 = new PathPoint(*(points.getGroups().at(0)->getPoints().at(2)),
                                                      PathPoint::WAIT, 500);

                path1.push_back(pathPoint1);
                path1.push_back(pathPoint2);
                path1.push_back(pathPoint3);

                path1.push_back(pathPoint1);
                path1.push_back(pathPoint2);
                path1.push_back(pathPoint3);

                path1.push_back(pathPoint1);
                path1.push_back(pathPoint2);
                path1.push_back(pathPoint3);
            }
        }
    }


    robot1->setPath(path1);

    RobotView* robotView1 = new RobotView(robot1);
    connect(robotView1, SIGNAL(setSelectedSignal(RobotView*)), this, SLOT(setSelectedRobot(RobotView*)));
    robotView1->setPosition(200, 200);
    robotView1->setParentItem(mapPixmapItem);
    robots->add(robotView1);

    Robot* robot2 = new Robot("Roboto", "192.168.4.176", PORT_CMD, this);
    robot2->setWifi("Swaghetti Yolognaise");
    RobotView* robotView2 = new RobotView(robot2);
    connect(robotView2, SIGNAL(setSelectedSignal(RobotView*)), this, SLOT(setSelectedRobot(RobotView*)));
    robotView2->setPosition(100, 100);
    robotView2->setParentItem(mapPixmapItem);
    robots->add(robotView2);

    Robot* robot3 = new Robot("Robota", "192.168.4.155", PORT_CMD, this);
    robot3->setWifi("Swaghetti Yolognaise");
    RobotView* robotView3 = new RobotView(robot3);
    connect(robotView3, SIGNAL(setSelectedSignal(RobotView*)), this, SLOT(setSelectedRobot(RobotView*)));
    robotView3->setPosition(200, 300);
    robotView3->setParentItem(mapPixmapItem);
    robots->add(robotView3);
}

void MainWindow::initializePoints(){

    XMLParser pParser(":/xml/points.xml");
    pParser.readPoints(points);
    pointViews = new PointsView(points);
    for(size_t j = 0; j < pointViews->getGroups().size(); j++){
        for(size_t k = 0; k < pointViews->getGroups().at(j).getPointViews().size(); k++){
            connect(&(*(pointViews->getGroups().at(j).getPointViews().at(k))),
                    SIGNAL(addPointPath(PointView*)), this,
                    SLOT(addPathPoint(PointView*)));
        }
    }
}

void MainWindow::initializeBottomPanel(){
    bottomLayout = new BottomLayout(this, robots);
    rightLayout->addWidget(bottomLayout);
}

void MainWindow::quit(){
    close();
}

void MainWindow::setSelectedRobot(RobotView* robotView){
    leftMenu->show();
    if(leftMenu->getRobotsLeftWidget()->getEditBtnStatus()){
        editSelectedRobot(robotView);
    } else {
        selectedRobot = robotView;
        robots->setSelected(robotView);

        hideAllWidgets();
        selectedRobotWidget->setSelectedRobot(selectedRobot);
        selectedRobotWidget->show();
    }
}

void MainWindow::stopSelectedRobot(int robotNb){
    qDebug() << "stopSelectedRobot called on robot : " << robots->getRobotsVector().at(robotNb)->getRobot()->getName();

    if(robots->getRobotsVector().at(robotNb)->getRobot()->getPath().size() > 0){

        QMessageBox msgBox;
        msgBox.setText("Are you sure you want to delete this path ?");
        msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Cancel);
        int ret = msgBox.exec();

        switch (ret) {
            case QMessageBox::Ok:
                qDebug() << "Points size before : " << points.getGroups().at(0)->getPoints().size();

                qDebug() << "Ok was clicked";

                /// if the command is succesfully sent to the robot, we apply the change
                if(robots->getRobotsVector().at(robotNb)->getRobot()->sendCommand(QString("s"))){
                    if(robots->getRobotsVector().at(robotNb)->getRobot()->isPlayingPath()){
                        qDebug() << "pause path on robot before supp " << robotNb << " : " << robots->getRobotsVector().at(robotNb)->getRobot()->getName();
                        robots->getRobotsVector().at(robotNb)->getRobot()->setPlayingPath(0);
                        bottomLayout->getPlayRobotBtnGroup()->button(robotNb)->setIcon(QIcon(":/icons/play.png"));
                    }
                    robots->getRobotsVector().at(robotNb)->getRobot()->getPath().clear();
                    robots->getRobotsVector().at(robotNb)->getRobot()->setPath(QVector<PathPoint*>());
                    qDebug() << "Path suppr, new path size : " << robots->getRobotsVector().at(robotNb)->getRobot()->getPath().size();
                    qDebug() << "Points size after : " << points.getGroups().at(0)->getPoints().size();
                    if(robots->getRobotsVector().at(robotNb)->getRobot()->getName().compare(selectedRobot->getRobot()->getName()) == 0){
                        hideAllWidgets();
                        selectedRobotWidget->setSelectedRobot(selectedRobot);
                        selectedRobotWidget->show();
                    }
                    bottomLayout->deletePath(robotNb);
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
        qDebug() << "This robot has no path";
    }
}

void MainWindow::playSelectedRobot(int robotNb){
    if(robots->getRobotsVector().at(robotNb)->getRobot()->isPlayingPath()){
        qDebug() << "pause path on robot " << robotNb << " : " << robots->getRobotsVector().at(robotNb)->getRobot()->getName();
        /// if the command is succesfully sent to the robot, we apply the change
        if(robots->getRobotsVector().at(robotNb)->getRobot()->sendCommand(QString("s"))){
            robots->getRobotsVector().at(robotNb)->getRobot()->setPlayingPath(0);
            bottomLayout->getPlayRobotBtnGroup()->button(robotNb)->setIcon(QIcon(":/icons/play.png"));
        }
    } else {
        qDebug() << "play path on robot " << robotNb << " : " << robots->getRobotsVector().at(robotNb)->getRobot()->getName();

        /// if the command is succesfully sent to the robot, we apply the change
        if(robots->getRobotsVector().at(robotNb)->getRobot()->sendCommand(QString("p"))){
            robots->getRobotsVector().at(robotNb)->getRobot()->setPlayingPath(1);
            bottomLayout->getPlayRobotBtnGroup()->button(robotNb)->setIcon(QIcon(":/icons/pause.png"));
        }
    }
}

void MainWindow::editSelectedRobot(RobotView* robotView){
    selectedRobot = robotView;
    robots->setSelected(robotView);

    robotsLeftWidget->setEditBtnStatus(false);
    robotsLeftWidget->setCheckBtnStatus(false);

    hideAllWidgets();
    editSelectedRobotWidget->setSelectedRobot(selectedRobot);
    editSelectedRobotWidget->show();
}

void MainWindow::setSelectedPoint(PointView* pointView, bool isTemporary){
    leftMenu->show();
    selectedPoint = pointView;

    hideAllWidgets();
    editSelectedPointWidget->setSelectedPoint(selectedPoint, isTemporary);
    editSelectedPointWidget->show();
    leftMenu->getDisplaySelectedPoint()->hide();
}

void MainWindow::robotBtnEvent(){
    qDebug() << "robotBtnEvent called";
    leftMenuWidget->hide();
    robotsLeftWidget->show();
    lastWidget = robotsLeftWidget;
}

void MainWindow::pointBtnEvent(){
    qDebug() << "pointBtnEvent called";
    hideAllWidgets();
    pointsLeftWidget->show();
    pointsLeftWidget->getGroupButtonGroup()->show();
    qDebug() << pointsLeftWidget->getGroupNameEdit()->pos().x() << " " << pointsLeftWidget->getGroupNameEdit()->pos().y();
    qDebug() << pointsLeftWidget->getGroupNameLabel()->pos().x() << " " << pointsLeftWidget->getGroupNameLabel()->pos().y();
    lastWidget = pointsLeftWidget;
}

void MainWindow::mapBtnEvent(){
    qDebug() << "mapBtnEvent called";
    leftMenuWidget->hide();
    mapLeftWidget->show();
    lastWidget = mapLeftWidget;
}

void MainWindow::backGroupBtnEvent(){
    qDebug() << "backPointBtnEvent called";
    pointsLeftWidget->hide();
    leftMenuWidget->show();
}

void MainWindow::plusGroupBtnEvent(){
    qDebug() << "plusGroupBtnEvent called";
    /// here we allow a user to create a new group
    pointsLeftWidget->getGroupNameEdit()->show();
    pointsLeftWidget->getGroupNameLabel()->show();

}

void MainWindow::minusGroupBtnEvent(){
    qDebug() << "minusPointBtnEvent called";
    int checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedId();
    qDebug() << checkedId;
    /// we have to delete a group
    if(checkedId > -1 && checkedId < points.getGroups().size()-1){
        askForDeleteGroupConfirmation(checkedId);
    }
    /// we have to delete a point
    else if(checkedId >= points.getGroups().size()-1){

        askForDeleteDefaultGroupPointConfirmation(checkedId-points.getGroups().size()+1);
    }
}

void MainWindow::editPointButtonEvent(){
    qDebug() << "editPointBtnEvent called";
    if(leftMenu->getDisplaySelectedPoint()->getEditButton()->isChecked())
        leftMenu->getDisplaySelectedPoint()->getNameEdit()->setReadOnly(false);
    else
        leftMenu->getDisplaySelectedPoint()->getNameEdit()->setReadOnly(true);
}

void MainWindow::editGroupBtnEvent(){
    qDebug() << "editPointBtnEvent called";

}

void MainWindow::selectPointBtnEvent(){
    qDebug() << "selectPointBtnEvent called";
}

//TODO add all the menu
void MainWindow::openLeftMenu(){
    qDebug() << "openLeftMenu called";
    leftMenu->getDisplaySelectedPoint()->hide();
    if(leftMenuWidget->isHidden()){
        robotsLeftWidget->setEditBtnStatus(false);
        robotsLeftWidget->setCheckBtnStatus(false);

        hideAllWidgets();
        leftMenuWidget->show();
        leftMenu->show();
        lastWidget = leftMenuWidget;
    } else {
        leftMenuWidget->hide();
        leftMenu->hide();
    }
}

void MainWindow::backSelecRobotBtnEvent(){
    qDebug() << "backSelecRobotBtnEvent called";
    selectedRobotWidget->hide();
    if(lastWidget != NULL){
        lastWidget->show();
    } else {
        leftMenu->hide();
    }
}

void MainWindow::editSelecRobotBtnEvent(){
    qDebug() << "editSelecRobotBtnEvent called";
    editSelectedRobot(selectedRobot);
}

void MainWindow::addPathSelecRobotBtnEvent(){
    qDebug() << "addPathSelecRobotBtnEvent called on robot " << selectedRobot->getRobot()->getName();
    hideAllWidgets();
    pathPainter->reset();
    pathCreationWidget->show();
    pathCreationWidget->resetWidget();
    pathCreationWidget->setSelectedRobot(selectedRobot->getRobot());
    setGraphicItemsState(GraphicItemState::CREATING_PATH, true);
}

void MainWindow::setSelectedRobot(QAbstractButton *button){
    qDebug() << "Edit : " << robotsLeftWidget->getEditBtnStatus() << "\nsetSelectedRobot with QAbstractButton called : " << button->text();

    if(robotsLeftWidget->getEditBtnStatus()){
        editSelectedRobot(robots->getRobotViewByName(button->text()));
    } else {
        setSelectedRobot(robots->getRobotViewByName(button->text()));
    }
}

void MainWindow::backRobotBtnEvent(){
    qDebug() << "backRobotBtnEvent called";
    robotsLeftWidget->setEditBtnStatus(false);
    robotsLeftWidget->setCheckBtnStatus(false);

    robotsLeftWidget->hide();
    leftMenuWidget->show();
}

void MainWindow::editRobotBtnEvent(){
    qDebug() << "editRobotBtnEvent called";
    if(robotsLeftWidget->getCheckBtnStatus()){
        robotsLeftWidget->setCheckBtnStatus(false);
        robotsLeftWidget->getBtnCheckGroup()->hide();
        robotsLeftWidget->getBtnGroup()->show();
    }
}

void MainWindow::checkRobotBtnEvent(){
    qDebug() << "checkRobotBtnEvent called";
    if(robotsLeftWidget->getEditBtnStatus())
        robotsLeftWidget->setEditBtnStatus(false);

    if(robotsLeftWidget->getCheckBtnStatus()){
        robotsLeftWidget->getBtnGroup()->hide();
        robotsLeftWidget->getBtnCheckGroup()->show();
    } else {
        robotsLeftWidget->getBtnCheckGroup()->hide();
        robotsLeftWidget->getBtnGroup()->show();
    }
}

void MainWindow::saveMapBtnEvent(){
    qDebug() << "saveMapBtnEvent called";
    QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
        "", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    qDebug() << "Dir name :" <<  dir;
    if(dir != ""){
        map->saveToFile(QString("map_saved.pgm"));
    } else {
        qDebug() << "Please select a directory";
    }
}

void MainWindow::loadMapBtnEvent(){
    qDebug() << "loadMapBtnEvent called";
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Image"), "", tr("Image Files (*.pgm)"));
    qDebug() << "File name :" << fileName;
    if(fileName != ""){
        map->setMapFromFile(fileName);
        QPixmap pixmap = QPixmap::fromImage(map->getMapImage());
        mapPixmapItem->setPixmap(pixmap);
        scene->update();
    }
}

void MainWindow::backMapBtnEvent(){
    qDebug() << "backMapBtnEvent called";
    mapLeftWidget->hide();
    leftMenuWidget->show();
}

void MainWindow::setCheckedRobot(QAbstractButton* button, bool checked){
    qDebug() << "setCheckedRobot called" << button->text();
    if(checked){
        qDebug() << "has been checked";
        robots->getRobotViewByName(button->text())->display(true);
    } else {
        qDebug() << "has been unchecked";
        robots->getRobotViewByName(button->text())->display(false);
    }
}

void MainWindow::cancelEditSelecRobotBtnEvent(){
    qDebug() << "cancelEditSelecRobotBtnEvent called";
    robotsLeftWidget->setEditBtnStatus(false);
    robotsLeftWidget->setCheckBtnStatus(false);
    editSelectedRobotWidget->hide();
    if(lastWidget != NULL){
        lastWidget->show();
    }
}

//TODO connect to robot
void MainWindow::robotSavedEvent(){
    qDebug() << "robotSavedEvent called";
    /// if the command is succesfully sent to the robot, we apply the change
    if(selectedRobot->getRobot()->sendCommand(QString(" n ") + editSelectedRobotWidget->getNameEdit()->text())){
        editSelectedRobotWidget->editName();
        robotsLeftWidget->setEditBtnStatus(false);
        robotsLeftWidget->setCheckBtnStatus(false);
        editSelectedRobotWidget->hide();
        if(lastWidget != NULL){
            lastWidget->show();
        }
        robotsLeftWidget->updateRobots(robots);
        bottomLayout->updateRobot(robots->getRobotId(selectedRobot->getRobot()->getName()), selectedRobot);
    }
}

void MainWindow::backSelecPointBtnEvent(){
    qDebug() << "backSelecPointBtnEvent called";
    selectedPointWidget->hide();
    if(lastWidget != NULL){
        lastWidget->show();
    } else {
        leftMenu->hide();
    }
}

void MainWindow::minusSelecPointBtnEvent(){
    qDebug() << "minusSelecPointBtnEvent called";
}

void MainWindow::editSelecPointBtnEvent(){
    qDebug() << "editSelecPointBtnEvent called";
}

void MainWindow::hideAllWidgets(){
    leftMenuWidget->hide();
    pointsLeftWidget->hide();
    selectedRobotWidget->hide();
    robotsLeftWidget->hide();
    mapLeftWidget->hide();
    editSelectedRobotWidget->hide();
    selectedPointWidget->hide();
    editSelectedPointWidget->hide();
    leftMenu->getDisplaySelectedPoint()->hide();
    pathCreationWidget->hide();
    leftMenu->getDisplaySelectedGroup()->hide();
}

/*
void MainWindow::cancelEditSelecPointBtnEvent(){
    qDebug() << "cancelEditSelecPointBtnEvent called";
    pointsLeftWidget->setEditBtnStatus(false);
    pointsLeftWidget->setCheckBtnStatus(false);
    //editSelectedPointWidget->hide();
    //if(lastWidget != NULL){
        //lastWidget->show();
    //}
}
*/

void MainWindow::pointSavedEvent(){
    /*
    qDebug() << "pointSavedEvent called";

    editSelectedPointWidget->hide();
    if(lastWidget != NULL){
        lastWidget->show();
    }
    // inform the user that a group must be chosen
    if(editSelectedPointWidget->getCurrentGroupIndex() == -1){
        QMessageBox messageBox;
        messageBox.setText("You have to choose a group for the point that you want to create");
        messageBox.setInformativeText("To do so, simply click on one of the groups of the list");
        messageBox.setStandardButtons(QMessageBox::Ok);
        messageBox.setIcon(QMessageBox::Information);
        messageBox.exec();
    } else {
        qDebug() << editSelectedPointWidget->getCurrentGroupIndex();
        qDebug() << points.getGroups().size();
        if(points.getGroups().at(points.getGroups().size()-editSelectedPointWidget->getCurrentGroupIndex()-1)->
                addPoint(Point(editSelectedPointWidget->getNameEdit()->text(), selectedPoint->getPoint()->getPosition().getX(), selectedPoint->getPoint()->getPosition().getY())) == 1){
            XMLParser pParser("/home//Documents/QtProject/gobot-software/points.xml");
            pParser.save(points);
            QMessageBox messageBox;
            leftMenu->getPointsLeftWidget()->getGroupMenu()->updateList(points);
            messageBox.setText("A new point has been added");
            messageBox.setStandardButtons(QMessageBox::Ok);
            messageBox.setIcon(QMessageBox::Information);
            messageBox.exec();
        } else if(points.getGroups().at(points.getGroups().size()-editSelectedPointWidget->getCurrentGroupIndex()-1)->
                   addPoint(Point(editSelectedPointWidget->getNameEdit()->text(), selectedPoint->getPoint()->getPosition().getX(), selectedPoint->getPoint()->getPosition().getY())) == 0){
            QMessageBox warningBox;
            warningBox.setText("This point could not be added because there is already a point with this name in the group");
            warningBox.setInformativeText("Please choose a new name for your point or place it in a different group");
            warningBox.setStandardButtons(QMessageBox::Ok);
            warningBox.setIcon(QMessageBox::Critical);
            warningBox.exec();
        } else {
            QMessageBox noNameBox;
            noNameBox.setText("You need to give a name to your point in order to add it permanently");
            noNameBox.setStandardButtons(QMessageBox::Ok);
            noNameBox.setIcon(QMessageBox::Critical);
            noNameBox.exec();
        }
    }
    qDebug() << editSelectedPointWidget->getCurrentGroupIndex();
    */
}

void MainWindow::selectHomeEvent(){
    qDebug() << "selectHomeEvent called";
}

void MainWindow::showHomeEvent(){
    qDebug() << "showHomeEvent called";
}

void MainWindow::displayDeleteEvent(QModelIndex index){
/*
    qDebug() << "displayMenu called on row " << index.row();
    qDebug() << pointsLeftWidget->getGroupDisplayed();
    if(index.row() < points.getGroups().size()-1){
        if(pointsLeftWidget->getGroupDisplayed()){
            pointsLeftWidget->setIndexLastGroupClicked(index.row());
            if(!pointsLeftWidget->getMinusBtn()->isChecked()){

                    pointsLeftWidget->getGroupMenu()->hide();
                    pointsLeftWidget->getBackButton()->hide();
                    pointsLeftWidget->getPointList()->setNewGroupToDisplay(index.row());
                    pointsLeftWidget->getPointList()->show();
                    pointsLeftWidget->getBackToGroupsBtn()->show();
                    pointsLeftWidget->setGroupDisplayed(false);

            } else {
                askForDeleteGroupConfirmation(index.row());
            }
        } else {
            if(pointsLeftWidget->getMinusBtn()->isChecked()){
                qDebug() << "wanna destroy a point";
                askForDeletePointConfirmation(index.row());
            }
        }
    }*/
}

void MainWindow::backToGroupsButtonEvent(void){
    /*
    qDebug() << "back to groups event called";
    //pointsLeftWidget->getPointList()->hide();
    pointsLeftWidget->getPointButtonGroup()->hide();
    pointsLeftWidget->getGroupMenu()->show();
    pointsLeftWidget->getBackToGroupsBtn()->hide();
    pointsLeftWidget->getBackButton()->show();
    pointsLeftWidget->getMinusBtn()->setChecked(false);
    // we display the groups again -> to make sure that the
    // minus button exhibits the right behavior
    pointsLeftWidget->setGroupDisplayed(true);
    */

}

void MainWindow::askForDeleteDefaultGroupPointConfirmation(int index){
    qDebug() << "point " << index;
    QMessageBox messageBox;
    messageBox.setText("Do you really want to remove this point ?");
    messageBox.setInformativeText("Be careful, the changes would be permanent");
    messageBox.setStandardButtons(QMessageBox::No | QMessageBox::Yes);
    messageBox.setDefaultButton(QMessageBox::Yes);
    messageBox.setIcon(QMessageBox::Question);
    int ret = messageBox.exec();

    switch(ret){
        case QMessageBox::No :
            qDebug() << "clicked no";
            pointsLeftWidget->getMinusButton()->setChecked(false);
        break;
        case QMessageBox::Yes : {
            pointsLeftWidget->getMinusButton()->setChecked(false);
            qDebug() << " called yes event ";
            points.getGroups().at(points.getGroups().size()-1)->removePoint(index);
            XMLParser parserPoints("/home/joan/Qt/QtProjects/gobot-software/gobot-software/points.xml");
            parserPoints.save(points);
            std::cout << pointViews->getGroups().at(pointViews->getGroups().size()-1).getPointViews().size() << std::endl;
            //pointViews->deleteDefaultPointView(index);
            std::cout << pointViews->getGroups().at(pointViews->getGroups().size()-1).getPointViews().size() << std::endl;
            pointsLeftWidget->getGroupButtonGroup()->update(points);
            /// need to remove the point from the map
            mapPixmapItem->updatePoints(points);
        }
        break;
        default:
        // should never be here
            qDebug() << " dafuk ?";
        break;
    }
}

void MainWindow::askForDeletePointConfirmation(int index){
    qDebug() << "point " << index;
    QMessageBox messageBox;
    messageBox.setText("Do you really want to remove this point ?");
    messageBox.setInformativeText("Be careful, the changes would be permanent");
    messageBox.setStandardButtons(QMessageBox::No | QMessageBox::Yes);
    messageBox.setDefaultButton(QMessageBox::Yes);
    messageBox.setIcon(QMessageBox::Question);
    int ret = messageBox.exec();

    switch(ret){
        case QMessageBox::No :
            qDebug() << "clicked no";
        break;
        case QMessageBox::Yes : {
            qDebug() << " called yes event ";
            points.getGroups().at(pointsLeftWidget->getIndexLastGroupClicked())->removePoint(index);
            XMLParser parserPoints("/home/joan/Qt/QtProjects/gobot-software/gobot-software/points.xml");
            parserPoints.save(points);
        }
        break;
        default:
        // should never be here
            qDebug() << " dafuk ?";
        break;
    }
}

void MainWindow::askForDeleteGroupConfirmation(int index){
    qDebug() << "group " << index;
    std::cout << *(points.getGroups().at(index)) << std::endl;
    QMessageBox messageBox;
    messageBox.setText("Do you really want to remove this group ? All the points in this group would also be removed.");
    messageBox.setInformativeText("Be careful, the changes would be permanent");
    messageBox.setStandardButtons(QMessageBox::No | QMessageBox::Yes);
    messageBox.setDefaultButton(QMessageBox::Yes);
    messageBox.setIcon(QMessageBox::Question);
    int ret = messageBox.exec();

    switch(ret){
        case QMessageBox::No :
            qDebug() << "clicked no";
            pointsLeftWidget->getMinusButton()->setChecked(false);
        break;
        case QMessageBox::Yes : {
            points.removeGroup(index);
            qDebug() << points.getGroups().size();
            XMLParser parserPoints("/home/joan/Qt/QtProjects/gobot-software/gobot-software/points.xml");
            parserPoints.save(points);
            std::cout << std::endl;
            mapPixmapItem->updatePoints(points);
            pointsLeftWidget->getGroupButtonGroup()->update(points);
            pointsLeftWidget->getMinusButton()->setChecked(false);
        }
        break;
        default:
            qDebug() << " dafuk ?";
        break;
    }
}

void MainWindow::displayPointEvent(PointView* _pointView){
    qDebug() << "ok";
    qDebug() << _pointView->getPoint()->getName();

    //leftMenu->getDisplaySelectedPoint()->displayPointInfo(_pointView->getPoint());
    leftMenu->getDisplaySelectedPoint()->setPoint(_pointView->getPoint());
    leftMenu->getDisplaySelectedPoint()->displayPointInfo();

    leftMenu->show();
    hideAllWidgets();
    leftMenu->getDisplaySelectedPoint()->show();
}

void MainWindow::modifyGroupEvent(int groupIndex){
    /*
    qDebug() << groupIndex;
    if(groupIndex < points.getGroups().size()-1)
        qDebug() << "modifyGroupEvent";
        */
}

void MainWindow::displayGroupEvent(int index, bool display){
    qDebug() << display;
    qDebug() << index << " " << points.getGroups().size()-1;
    if(pointsLeftWidget->getMapButton()->isChecked()){
        if(display){
            if(index < points.getGroups().size()-1){
                qDebug() << " i have to display a whole group ";
                for(int i = 0; i < points.getGroups().at(index)->getPoints().size(); i++){
                    std::shared_ptr<Point> currentPoint = points.getGroups().at(index)->getPoints().at(i);
                    currentPoint->setDisplayed(true);
                    mapPixmapItem->updatePoints(points);
                    XMLParser parserPoints("/home/joan/Qt/QtProjects/gobot-software/gobot-software/points.xml");
                    parserPoints.save(points);
                }
            } else {
                qDebug() << " i have to display a single point";
                points.getGroups().at(points.getGroups().size()-1)->getPoints().at(index-points.getGroups().size()+1)->setDisplayed(true);
                mapPixmapItem->updatePoints(points);
                XMLParser parserPoints("/home/joan/Qt/QtProjects/gobot-software/gobot-software/points.xml");
                parserPoints.save(points);

            }
        } else {
            if(index < points.getGroups().size()-1){
                qDebug() << " i have to stop displaying a whole group ";
                for(int i = 0; i < points.getGroups().at(index)->getPoints().size(); i++){
                    std::shared_ptr<Point> currentPoint = points.getGroups().at(index)->getPoints().at(i);
                    currentPoint->setDisplayed(false);
                    mapPixmapItem->updatePoints(points);
                    XMLParser parserPoints("/home/joan/Qt/QtProjects/gobot-software/gobot-software/points.xml");
                    parserPoints.save(points);
                }

            } else {
                qDebug() << " i have to stop displaying a single point which index is " << index-points.getGroups().size()+1;
                points.getGroups().at(points.getGroups().size()-1)->getPoints().at(index-points.getGroups().size()+1)->setDisplayed(false);
                mapPixmapItem->updatePoints(points);
                XMLParser parserPoints("/home/joan/Qt/QtProjects/gobot-software/gobot-software/points.xml");
                parserPoints.save(points);
            }
        }
    }
}

void MainWindow::displayGroupMapEvent(){
    pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->setExclusive(false);

    qDebug() << "displaying groups by clicking on the map button";
    if(pointsLeftWidget->getMapButton()->isChecked())
        pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->setExclusive(false);
    else
        pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->setExclusive(true);
    qDebug() << " heere ";

}

void MainWindow::displayPointMapEvent(){

    std::shared_ptr<Point> point = leftMenu->getDisplaySelectedPoint()->getPoint();

    if(point != NULL && point->isDisplayed()){
        qDebug() << " I was displayed, but it's over";
        point->setDisplayed(false);
        mapPixmapItem->updatePoints(points);
    } else if (point != NULL && !point->isDisplayed()){
        qDebug() << " Now I have returned to be displayed";
        point->setDisplayed(true);
        mapPixmapItem->updatePoints(points);
    } else {
        qDebug() << "wtf am I manipulating a NULL pointer ?";
    }
}

void MainWindow::removeGroupEvent(const int groupIndex){
    if(pointsLeftWidget->getMinusButton()->isChecked()){
        int checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedId();
        qDebug() << checkedId;
        /// we have to delete a group
        if(checkedId > -1 && checkedId < points.getGroups().size()-1){
            askForDeleteGroupConfirmation(checkedId);
        }
        /// we have to delete a point
        else if(checkedId >= points.getGroups().size()-1){

            askForDeleteDefaultGroupPointConfirmation(checkedId-points.getGroups().size()+1);
        }
    }
}

void MainWindow::backPathCreation(void){
    qDebug() << "backPathCreation called";
    hideAllWidgets();
    selectedRobotWidget->show();
}

void MainWindow::setGraphicItemsState(const GraphicItemState state, const bool clear){
    mapPixmapItem->setState(state, clear);

    for(int i = 0; i < robots->getRobotsVector().size(); i++){
        robots->getRobotsVector().at(i)->setState(state);
    }

    for(size_t j = 0; j < pointViews->getGroups().size(); j++){
        GroupView groupView = pointViews->getGroups().at(j);
        for(size_t k = 0; k < groupView.getPointViews().size(); k++){
            groupView.getPointViews().at(k)->setState(state);
        }
    }
}

void MainWindow::pathSaved(void){
    qDebug() << "pathSaved called";
    hideAllWidgets();
    selectedRobotWidget->setSelectedRobot(selectedRobot);
    selectedRobotWidget->show();
    bottomLayout->updateRobot(robots->getRobotId(selectedRobot->getRobot()->getName()), selectedRobot);
}

void MainWindow::addPathPoint(Point* point){
    qDebug() << "addPathPoint called on point" << point->getName();
    pathCreationWidget->addPathPoint(point);
}

void MainWindow::addPathPoint(PointView* pointView){
    qDebug() << "addPathPoint called on point" << pointView->getPoint()->getName();
    pathCreationWidget->addPathPoint(&(*(pointView->getPoint())));
}


void MainWindow::displayPointsInGroup(void){
    int groupIndex = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedId();
    /// it's a group
    if(groupIndex != -1 && groupIndex < points.getGroups().size()-1){
       pointsLeftWidget->getEyeButton()->setChecked(false);
       pointsLeftWidget->hide();
       /// before we display the group of points, we make sure that the graphical object is consistent with the model
       leftMenu->updateGroupDisplayed(points, groupIndex);
       leftMenu->getDisplaySelectedGroup()->show();
       leftMenu->getDisplaySelectedGroup()->setName(points.getGroups().at(groupIndex)->getName());
    }
    /// it's an isolated point
    else if(groupIndex >= points.getGroups().size()-1){

        //leftMenu->getDisplaySelectedPoint()->displayPointInfo(
        //            points.getGroups().at(points.getGroups().size()-1)->getPoints().at(groupIndex-points.getGroups().size()+1));

        leftMenu->getDisplaySelectedPoint()->setPoint(points.getGroups().at(points.getGroups().size()-1)->getPoints().at(groupIndex+1-points.getGroups().size()));
        leftMenu->getDisplaySelectedPoint()->displayPointInfo();

        leftMenu->getDisplaySelectedPoint()->show();

        pointsLeftWidget->getEyeButton()->setChecked(false);
        pointsLeftWidget->hide();
    }
}

void MainWindow::updatePathPointToPainter(QVector<Point>* pointVector){
    pathPainter->updatePath(*pointVector);
}

void MainWindow::stopPathCreation(){
    for(size_t i = 0; i < pointViews->getGroups().size(); i++){
        GroupView groupView = pointViews->getGroups().at(i);
        std::vector<std::shared_ptr<PointView>> pointViews = groupView.getPointViews();
        for(size_t j = 0; j < pointViews.size(); j++){
            pointViews.at(j)->setPixmap(QPixmap(":/icons/cropped_coordinates"));
        }
    }
}

void MainWindow::hidePathCreationWidget(){
    qDebug() << "hidePathCreationWidget called";
    setGraphicItemsState(GraphicItemState::NO_STATE, true);
    for(size_t i = 0; i < pointViews->getGroups().size(); i++){
        GroupView groupView = pointViews->getGroups().at(i);
        std::vector<std::shared_ptr<PointView>> pointViews = groupView.getPointViews();
        for(size_t j = 0; j < pointViews.size(); j++){
            pointViews.at(j)->setPixmap(QPixmap(PIXMAP_NORMAL));
            pointViews.at(j)->setAddedToPath(false);
        }
    }
    pathCreationWidget->resetWidget();
    pathPainter->reset();
}

void MainWindow::removePointFromInformationMenu(void){
    QMessageBox messageBox;
    messageBox.setText("Do you really want to remove this point ?");
    messageBox.setInformativeText("Be careful, the changes would be permanent");
    messageBox.setStandardButtons(QMessageBox::No | QMessageBox::Yes);
    messageBox.setDefaultButton(QMessageBox::Yes);
    messageBox.setIcon(QMessageBox::Question);
    int ret = messageBox.exec();

    switch(ret){
        case QMessageBox::No :
            qDebug() << "clicked no";
            pointsLeftWidget->getMinusButton()->setChecked(false);
        break;
        case QMessageBox::Yes : {
            /// to get the name of the point we just retrieve the label text property without the first 7 chars "Name : "
            QString pointName = leftMenu->getDisplaySelectedPoint()->getPointName();
            /// holds the index of the group and the index of a particular point in this group within <points>
            std::pair<int, int> pointIndexes = points.findPointIndexes(pointName);
            if(pointIndexes.first != -1){
                points.getGroups().at(pointIndexes.first)->removePoint(pointIndexes.second);
                /// updates the file containing containing points info
                XMLParser parserPoints("/home/joan/Qt/QtProjects/gobot-software/gobot-software/points.xml");
                parserPoints.save(points);
                /// need to remove the point from the map
                mapPixmapItem->updatePoints(points);
                /// updates the group menu and the list of points
                pointsLeftWidget->getGroupButtonGroup()->update(points);
                /// closes the window
                leftMenu->getDisplaySelectedPoint()->hide();

            } else {
                qDebug() << "could not find this point";
            }
        }
        break;
        default:
        // should never be here
            qDebug() << " dafuk ?";
        break;
    }
}

void MainWindow::pointInfoEvent(void){
    if(pointsLeftWidget->getEyeButton()->isChecked()){
        pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->setExclusive(true);
        int groupIndex = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedId();
        /// it's a group
        if(groupIndex != -1 && groupIndex < points.getGroups().size()-1){
           pointsLeftWidget->getEyeButton()->setChecked(false);
           pointsLeftWidget->hide();
           /// before we display the group of points, we make sure that the graphical object is consistent with the model
           leftMenu->updateGroupDisplayed(points, groupIndex);
           leftMenu->getDisplaySelectedGroup()->show();
           leftMenu->getDisplaySelectedGroup()->setName(points.getGroups().at(groupIndex)->getName());
        }
        /// it's an isolated point
        else if(groupIndex >= points.getGroups().size()-1){

            //leftMenu->getDisplaySelectedPoint()->displayPointInfo(
             //           points.getGroups().at(points.getGroups().size()-1)->getPoints().at(groupIndex-points.getGroups().size()+1));
            leftMenu->getDisplaySelectedPoint()->displayPointInfo();


            leftMenu->getDisplaySelectedPoint()->show();
            pointsLeftWidget->getEyeButton()->setChecked(false);
            pointsLeftWidget->hide();
        }
    }
}

void MainWindow::editTmpPathPointSlot(int id, Point* point, int nbWidget){
    qDebug() << "editTmpPathPointSlot called : " << id << point->getName() << nbWidget;
    if(nbWidget == 1){
        editedPointView = NULL;

        QVector<PointView*> pointViewVector = mapPixmapItem->getPathCreationPoints();
        for(int i = 0; i < pointViewVector.size(); i++){
            if(pointViewVector.at(i)->getPoint()->comparePos(point->getPosition().getX(), point->getPosition().getY())){
                editedPointView = pointViewVector.at(i);
            }
        }

        if(mapPixmapItem->getTmpPointView()->getPoint()->comparePos(point->getPosition().getX(), point->getPosition().getY())){
            editedPointView = mapPixmapItem->getTmpPointView();
        }

        if(editedPointView == NULL){
            qDebug() << "(Error editTmpPathPointSlot) No pointview found to edit";
        } else {
            qDebug() << "Pointview found";
                editedPointView->setFlag(QGraphicsItem::ItemIsMovable);
                setGraphicItemsState(GraphicItemState::NO_EVENT, false);
                editedPointView->setState(GraphicItemState::EDITING);
        }

    } else if(nbWidget > 1){
        // TODO

    } else {
        qDebug() << "(Error editTmpPathPointSlot) Not supposed to be here";
    }
}

void MainWindow::saveTmpEditPathPointSlot(){
    qDebug() << "saveTmpEditPathPointSlot called";
    pathCreationWidget->applySavePathPoint(editedPointView->getPoint()->getPosition().getX(), editedPointView->getPoint()->getPosition().getY());
    editedPointView->setFlag(QGraphicsItem::ItemIsMovable, false);
    setGraphicItemsState(GraphicItemState::CREATING_PATH, false);

    editedPointView = NULL;
}


void MainWindow::moveTmpEditPathPointSlot(){
    pathCreationWidget->moveEditPathPoint(editedPointView->getPoint()->getPosition().getX(), editedPointView->getPoint()->getPosition().getY());
}
