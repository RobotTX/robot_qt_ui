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
#include "Model/pathpoint.h"
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
#include "View/pointbuttongroup.h"
#include "View/verticalscrollarea.h"
#include "View/toplayout.h"
#include <QVBoxLayout>
#include <QAbstractButton>

//#define XML_PATH "/home/m-a/Documents/QtProject/gobot-software/points.xml"
#define XML_PATH "/home/joan/Qt/QtProjects/gobot-software/points.xml"
//#define XML_PATH "/Users/fannylarradet/Desktop/GTRobots/gobot-software/points.xml"

/**
 * @brief MainWindow::MainWindow
 * @param parent
 * The main controller of the application
 */
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    QWidget* mainWidget = new QWidget(this);

    QVBoxLayout* mainLayout = new QVBoxLayout(mainWidget);
    map = std::shared_ptr<Map>(new Map());

    QSettings settings;

    map->setMapFromFile(settings.value("mapFile", ":/maps/map.pgm").toString());

    robots = std::shared_ptr<Robots>(new Robots());
    scene = new QGraphicsScene(this);
    graphicsView = new CustomQGraphicsView(scene, this);
    selectedRobot = NULL;
    scanningRobot = NULL;
    selectedPoint = NULL;
    editedPointView = NULL;
    resetFocus();

    //create the graphic item of the map
    QPixmap pixmap = QPixmap::fromImage(map->getMapImage());
    mapPixmapItem = new MapView(pixmap, QSize(geometry().width(), geometry().height()), this);
    connect(mapPixmapItem, SIGNAL(addPathPointMapView(Point*)), this, SLOT(addPathPoint(Point*)));
    connect(mapPixmapItem, SIGNAL(homeSelected(PointView*, bool)), this, SLOT(homeSelected(PointView*, bool)));
    connect(mapPixmapItem, SIGNAL(homeEdited(PointView*, bool)), this, SLOT(homeEdited(PointView*, bool)));

    //create the toolbar
    topLayout = new TopLayout(this);
    mainLayout->addWidget(topLayout);

    QHBoxLayout* bottom = new QHBoxLayout();

    initializePoints();
    mapPixmapItem->setPermanentPoints(pointViews);

    pathPainter = new PathPainter(mapPixmapItem, pointViews);
    initializeRobots();

    scene->addItem(mapPixmapItem);

    // hide the scroll bars
    graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    graphicsView->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));

    leftMenu = new LeftMenu(this, points, robots, pointViews);
    initializeLeftMenu();
    bottom->addWidget(leftMenu);


    rightLayout = new QVBoxLayout();
    bottom->addLayout(rightLayout);
    rightLayout->addWidget(graphicsView);

    initializeBottomPanel();

    graphicsView->show();

    /// to link the map and the point information menu when a point is being edited
    connect(mapPixmapItem, SIGNAL(newCoordinates(double, double)), leftMenu->getDisplaySelectedPoint(), SLOT(updateCoordinates(double, double)));

    /// to cancel the modifications on an edited point
    connect(leftMenu->getDisplaySelectedPoint()->getCancelButton(), SIGNAL(clicked(bool)), this, SLOT(cancelEvent()));

    /// to save the modifications on an edited point
    connect(leftMenu->getDisplaySelectedPoint()->getSaveButton(), SIGNAL(clicked(bool)), this, SLOT(updatePoint()));

    /// the purpose of this connection is just to propagate the signal to the map view through the main window
    connect(leftMenu->getDisplaySelectedPoint(), SIGNAL(nameChanged(QString, QString)), this, SLOT(updatePoint()));

    /// to update the names of the points displayed when a user changes the name of a point via the edit button
    connect(this, SIGNAL(nameChanged(QString, QString)), mapPixmapItem, SLOT(updateHover(QString, QString)));

    /// to reset the state of everybody when a user click on a random button while he was editing a point
    connect(leftMenu->getDisplaySelectedPoint(), SIGNAL(resetState(GraphicItemState, bool)),  this, SLOT(setGraphicItemsState(GraphicItemState, bool)));

    /// to connect the buttons in the left menu so they can be double clicked after they were updated
    connect(pointsLeftWidget->getGroupButtonGroup(), SIGNAL(updateConnectionsRequest()), this, SLOT(reestablishConnectionsGroups()));

    /// to connect the buttons in the group menu so they can be double clicked after they were updated
    connect(leftMenu->getDisplaySelectedGroup()->getPointButtonGroup(), SIGNAL(updateConnectionsRequest()), this, SLOT(reestablishConnectionsPoints()));

    mainLayout->addLayout(bottom);
    setCentralWidget(mainWidget);
}

MainWindow::~MainWindow(){
    delete ui;
    delete pathPainter;
    qDeleteAll(pathPointViews.begin(), pathPointViews.end());
    pathPointViews.clear();
}



/**********************************************************************************************************************************/

//                                          ROBOTS and PATHS

/**********************************************************************************************************************************/

void MainWindow::updateRobot(const float posX, const float posY, const float oriZ){

    float newPosX = (-map->getOrigin().getX()+posX)/map->getResolution() + ROBOT_WIDTH;
    float newPosY = map->getHeight()-(-map->getOrigin().getY()+posY)/map->getResolution()-ROBOT_WIDTH/2;
    float ori = asin(-oriZ) * 360.0 / PI + 90;

    scanningRobot->setPosition(newPosX, newPosY);
    scanningRobot->setOrientation(ori);

    scene->update();

    qDebug() << "Robot position : " << scanningRobot->getRobot()->getPosition().getX()
             << " " << scanningRobot->getRobot()->getPosition().getY()
             << " " << scanningRobot->getRobot()->getOrientation();
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

                    if(selectedRobot->getRobot()->sendCommand(QString("e ") + QString::number(PORT_MAP_METADATA) + " " + QString::number(PORT_ROBOT_POS) + " " +QString::number(PORT_MAP))){
                        selectedRobotWidget->getScanBtn()->setText("Stop to scan");
                        clearNewMap();
                        selectedRobotWidget->disable();
                        selectedRobotWidget->getScanBtn()->setEnabled(true);
                        bottomLayout->disable();
                        setGraphicItemsState(GraphicItemState::NO_EVENT);
                        disableMenu();

                        metadataThread = new ScanMetadataThread(ip, PORT_MAP_METADATA);
                        robotThread = new ScanRobotThread(ip, PORT_ROBOT_POS);
                        mapThread = new ScanMapThread(ip, PORT_MAP);

                        connect(robotThread, SIGNAL(valueChangedRobot(float, float, float))
                                ,this ,SLOT(updateRobot(float, float, float)));

                        connect(metadataThread, SIGNAL(valueChangedMetadata(int, int, float, float, float))
                                , this , SLOT(updateMetadata(int, int, float, float, float)));

                        connect(mapThread, SIGNAL(valueChangedMap(QByteArray))
                                , this , SLOT(updateMap(QByteArray)));

                        metadataThread->start();
                        metadataThread->moveToThread(metadataThread);

                        robotThread->start();
                        robotThread->moveToThread(robotThread);

                        mapThread->start();
                        mapThread->moveToThread(mapThread);

                        scanningRobot = selectedRobot;
                    } else {
                        selectedRobotWidget->getScanBtn()->setChecked(false);
                    }
                }
                break;
                default:
                // should never be here
                    qDebug() << " dafuk ?";
                break;
            }
        } else {
            if(selectedRobot->getRobot()->sendCommand("f")){
                qDebug() << "Disconnected";
                selectedRobotWidget->getScanBtn()->setText("Scan a map");
                selectedRobotWidget->enable();
                bottomLayout->enable();
                setGraphicItemsState(GraphicItemState::NO_STATE);
                enableMenu();
            } else {
                qDebug() << "Could not disconnect";
                selectedRobotWidget->getScanBtn()->setChecked(true);
            }
        }
    } else {
        qDebug() << "Select a robot first";
    }
}

void MainWindow::initializeRobots(){
    //TODO Need to come from XML
    std::shared_ptr<Robot> robot1(new Robot("Roboty", "localhost", PORT_CMD, this));
    robot1->setWifi("Swaghetti Yolognaise");
    RobotView* robotView1 = new RobotView(robot1, mapPixmapItem);
    connect(robotView1, SIGNAL(setSelectedSignal(RobotView*)), this, SLOT(setSelectedRobot(RobotView*)));
    robotView1->setPosition(200, 200);
    robotView1->setParentItem(mapPixmapItem);
    robots->add(robotView1);

    std::shared_ptr<Robot> robot2(new Robot("Roboto", "192.168.4.176", PORT_CMD, this));
    robot2->setWifi("Swaghetti Yolognaise");
    RobotView* robotView2 = new RobotView(robot2, mapPixmapItem);
    connect(robotView2, SIGNAL(setSelectedSignal(RobotView*)), this, SLOT(setSelectedRobot(RobotView*)));
    robotView2->setPosition(100, 100);
    robotView2->setParentItem(mapPixmapItem);
    robots->add(robotView2);

    std::shared_ptr<Robot> robot3(new Robot("Robota", "192.168.4.175", PORT_CMD, this));
    robot3->setWifi("Swaghetti Yolognaise");
    RobotView* robotView3 = new RobotView(robot3, mapPixmapItem);
    connect(robotView3, SIGNAL(setSelectedSignal(RobotView*)), this, SLOT(setSelectedRobot(RobotView*)));
    robotView3->setPosition(200, 300);
    robotView3->setParentItem(mapPixmapItem);
    robots->add(robotView3);
}

void MainWindow::stopSelectedRobot(int robotNb){
    qDebug() << "stopSelectedRobot called on robot : " << robots->getRobotsVector().at(robotNb)->getRobot()->getName();

    if(robots->getRobotsVector().at(robotNb)->getRobot()->getPath().size() > 0){
        int ret = openConfirmMessage("Are you sure you want to delete this path ?");
        switch (ret) {
            case QMessageBox::Ok:
                qDebug() << "Points size before : " << points.getGroups().at(0)->getPoints().size();
                qDebug() << "Ok was clicked";
                /// if the command is succesfully sent to the robot, we apply the change
                if(robots->getRobotsVector().at(robotNb)->getRobot()->sendCommand(QString("d"))){
                    clearPath(robotNb);
                    qDebug() << "Path suppr, new path size : " << robots->getRobotsVector().at(robotNb)->getRobot()->getPath().size();
                    qDebug() << "Points size after : " << points.getGroups().at(0)->getPoints().size();
                    if(!robots->getRobotsVector().at(robotNb)->getRobot()->getName().compare(selectedRobot->getRobot()->getName())){
                        hideAllWidgets();
                        selectedRobotWidget->setSelectedRobot(selectedRobot);
                        selectedRobotWidget->show();
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
        qDebug() << "This robot has no path";
    }
}

void MainWindow::playSelectedRobot(int robotNb){
    std::shared_ptr<Robot> robot = robots->getRobotsVector().at(robotNb)->getRobot();
    if(robot->isPlayingPath()){
        qDebug() << "pause path on robot " << robotNb << " : " << robot->getName();
        /// if the command is succesfully sent to the robot, we apply the change
        if(robot->sendCommand(QString("d"))){
            robot->setPlayingPath(0);
            bottomLayout->getPlayRobotBtnGroup()->button(robotNb)->setIcon(QIcon(":/icons/play.png"));
        }
    } else {
        qDebug() << "play path on robot " << robotNb << " : " << robot->getName();
        std::shared_ptr<PathPoint> pathPoint = robot->getPath().at(0);
        float oldPosX = pathPoint->getPoint().getPosition().getX();
        float oldPosY = pathPoint->getPoint().getPosition().getY();
        qDebug() << "Go to next point :" << oldPosX << oldPosY;
        qDebug() << "ok1" << (float) oldPosX;
        qDebug() << "ok2" << (float) (oldPosX - ROBOT_WIDTH);
        qDebug() << "ok3" << (float) ((oldPosX - ROBOT_WIDTH) * map->getResolution());
        qDebug() << "ok4" << (float) ((oldPosX - ROBOT_WIDTH) * map->getResolution() + map->getOrigin().getX());

        float newPosX = (oldPosX - ROBOT_WIDTH) * map->getResolution() + map->getOrigin().getX();
        float newPosY = (-oldPosY + map->getHeight() - ROBOT_WIDTH/2) * map->getResolution() + map->getOrigin().getY();
        qDebug() << "Go to next point :" << newPosX << newPosY;
        int waitTime = -1;
        if(pathPoint->getAction() == PathPoint::WAIT){
            waitTime = pathPoint->getWaitTime();
        }

        /// if the command is succesfully sent to the robot, we apply the change
        if(robot->sendCommand(QString("c ") + QString::number(newPosX) + " "  + QString::number(newPosY) + " "  + QString::number(waitTime))){
            robot->setPlayingPath(1);
            bottomLayout->getPlayRobotBtnGroup()->button(robotNb)->setIcon(QIcon(":/icons/pause.png"));
        }
    }
}

void MainWindow::viewPathSelectedRobot(int robotNb){
    bool checked = bottomLayout->getViewPathRobotBtnGroup()->button(robotNb)->isChecked();
    if(checked){
        std::shared_ptr<Robot> robot = robots->getRobotsVector().at(robotNb)->getRobot();
        qDebug() << "viewPathSelectedRobot called on" << robot->getName() << checked;
        bottomLayout->uncheckViewPathSelectedRobot(robotNb);
        if(pathPointViews.size() > 0){
            qDeleteAll(pathPointViews.begin(), pathPointViews.end());
            pathPointViews.clear();
        }

        for(size_t i = 0; i < robot->getPath().size(); i++){
            std::shared_ptr<PathPoint> pathPoint = robot->getPath().at(i);
            PointView * pointView = new PointView(std::make_shared<Point>(pathPoint->getPoint()), mapPixmapItem);
            //pointView->setParentItem(mapPixmapItem);
            pathPointViews.push_back(pointView);
        }
        pathPainter->updatePath(pathPointViews);
    } else {
        if(pathPointViews.size() > 0){
            qDeleteAll(pathPointViews.begin(), pathPointViews.end());
            pathPointViews.clear();
        }
        pathPainter->reset();
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

void MainWindow::setSelectedRobot(RobotView* robotView){

    qDebug() << "setSelectedRobot(RobotView* robotView)";
    updateView();
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

void MainWindow::robotBtnEvent(void){
    qDebug() << "robotBtnEvent called";
    leftMenuWidget->hide();
    robotsLeftWidget->show();
    switchFocus("Robots", robotsLeftWidget);
}



void MainWindow::backSelecRobotBtnEvent(){
    qDebug() << "backSelecRobotBtnEvent called";
    selectedRobotWidget->hide();
    if(lastWidget.last() != NULL){
        lastWidget.last()->show();
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


void MainWindow::setSelectedRobotNoParent(QAbstractButton *button){

   // switchFocus("Robot", selectedRobotWidget);
    resetFocus();
    setSelectedRobot(button);
}


void MainWindow::setSelectedRobot(QAbstractButton *button){
    qDebug() << "Edit : " << robotsLeftWidget->getEditBtnStatus() << "\nsetSelectedRobot with QAbstractButton called : " << button->text();
    if(robotsLeftWidget->getEditBtnStatus())
        (robots->getRobotViewByName(button->text()));
    else
        setSelectedRobot(robots->getRobotViewByName(button->text()));
    switchFocus("Robot", selectedRobotWidget);

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

void MainWindow::cancelEditSelecRobotBtnEvent(){
    qDebug() << "cancelEditSelecRobotBtnEvent called";

    robotsLeftWidget->setEditBtnStatus(false);
    robotsLeftWidget->setCheckBtnStatus(false);
    editSelectedRobotWidget->hide();
    if(lastWidget.last() != NULL){
        lastWidget.last()->show();
    }
}

void MainWindow::robotSavedEvent(){
    qDebug() << "robotSavedEvent called";

    bool isOK = false;

    /// if the command is succesfully sent to the robot, we apply the change
    if (editSelectedRobotWidget->getWifiPwdEdit()->text() == "......")
    {
        qDebug() << "edit name only";

        if(selectedRobot->getRobot()->sendCommand(QString("a ") + editSelectedRobotWidget->getNameEdit()->text())){
            isOK = true;
        }
    }
    else
    {
        if(selectedRobot->getRobot()->sendCommand(QString("a ") + editSelectedRobotWidget->getNameEdit()->text())
                && selectedRobot->getRobot()->sendCommand(QString("b ")
              + editSelectedRobotWidget->getWifiNameEdit()->text()
              + editSelectedRobotWidget->getWifiPwdEdit()->text())){

            isOK=true;
        }
    }

    if (isOK){

        editSelectedRobotWidget->editName();

        PointView* pointView = editSelectedRobotWidget->getHome();
        if(pointView != NULL && !(&(*(pointView->getPoint())) == &(*(selectedRobot->getRobot()->getHome())))){
            int ret = openConfirmMessage("Do you really want to set the point " + pointView->getPoint()->getName() +
                                         + " (" + QString::number(pointView->getPoint()->getPosition().getX(),'f', 1) + ","
                                         + QString::number(pointView->getPoint()->getPosition().getY(),'f', 1) + ") as the home for "
                                         + selectedRobot->getRobot()->getName() +" ?");
            switch(ret){
                case QMessageBox::Cancel :
                    pointsLeftWidget->getMinusButton()->setChecked(false);
                    if(editSelectedRobotWidget->isTemporaryHome()){
                        delete pointView;
                    }
                break;
                case QMessageBox::Ok : {
                    bool done = false;
                    if(editSelectedRobotWidget->isTemporaryHome()){
                        qDebug() << "Tmp point";
                        if(points.count() > 0 && pointViews->getGroups().size() > 0){
                            pointView->getPoint()->setHome(true, selectedRobot->getRobot()->getName());
                            points.getGroups().at(points.count()-1)->addPoint(pointView->getPoint());
                            XMLParser parserPoints(XML_PATH, mapPixmapItem);
                            parserPoints.save(points);

                            pointViews->setPoints(points);
                            pointViews->getGroups().at(pointViews->getGroups().size()-1)->addPointView(pointView);
                            done = true;
                        }
                    } else {
                        qDebug() << "Permanent point";
                        if(pointView->getPoint()->setHome(true, selectedRobot->getRobot()->getName())){
                            done = true;
                        } else {
                            setMessageTop(TEXT_COLOR_DANGER, "Sorry, this point is already a home\nPlease select another");
                        }
                    }

                    pointsLeftWidget->updateGroupButtonGroup(points);

                    if(done){
                        setMessageTop(TEXT_COLOR_SUCCESS, selectedRobot->getRobot()->getName() + " now has a new home");

                        if(selectedRobot->getRobot()->getHome() != NULL)
                            selectedRobot->getRobot()->getHome()->setHome(false, "");

                        selectedRobot->getRobot()->setHome(editSelectedRobotWidget->getHome()->getPoint());
                    }
                }
                break;
                default:
                // should never be here
                    qDebug() << " dafuk ?";
                break;
            }
        }
        robotsLeftWidget->setEditBtnStatus(false);
        robotsLeftWidget->setCheckBtnStatus(false);
        editSelectedRobotWidget->hide();

        robotsLeftWidget->updateRobots(robots);
        bottomLayout->updateRobot(robots->getRobotId(selectedRobot->getRobot()->getName()), selectedRobot);

        selectedRobotWidget->setSelectedRobot(selectedRobot );
        selectedRobotWidget->show();
    }
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

void MainWindow::backPathCreation(void){
    qDebug() << "backPathCreation called";
    hideAllWidgets();
    selectedRobotWidget->show();
}

void MainWindow::editTmpPathPointSlot(int id, Point* point, int nbWidget){
    qDebug() << "editTmpPathPointSlot called : " << id << point->getName() << nbWidget;
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
        if(nbWidget == 1){
            editedPointView->setFlag(QGraphicsItem::ItemIsMovable);
            setGraphicItemsState(GraphicItemState::NO_EVENT, false);
            editedPointView->setState(GraphicItemState::EDITING);
        } else if(nbWidget > 1){
            mapPixmapItem->addPathPoint(editedPointView);
            editedPointView = mapPixmapItem->getPathCreationPoints().last();
            editedPointView->setFlag(QGraphicsItem::ItemIsMovable);
            setGraphicItemsState(GraphicItemState::NO_EVENT, false);
            editedPointView->setState(GraphicItemState::EDITING);
        } else {
            qDebug() << "(Error editTmpPathPointSlot) Not supposed to be here";
        }
    }
}

void MainWindow::pathSaved(bool execPath){
    qDebug() << "pathSaved called" << execPath;

    hideAllWidgets();
    setMessageTop(TEXT_COLOR_SUCCESS, "Path saved");
    selectedRobotWidget->setSelectedRobot(selectedRobot);
    selectedRobotWidget->show();
    bottomLayout->updateRobot(robots->getRobotId(selectedRobot->getRobot()->getName()), selectedRobot);

    if(execPath){
        int robotNb = -1;
        for(int i = 0; i < bottomLayout->getPlayRobotBtnGroup()->buttons().size(); i++){
            if(bottomLayout->getRobotBtnGroup()->button(i)->text().compare(selectedRobot->getRobot()->getName()) == 0){
                robotNb = i;
                qDebug() << "robotNb :" << robotNb;
            }
        }
        if(robotNb >= 0)
            playSelectedRobot(robotNb);
        else
            qDebug() << "No robot to play this path";
    }
}

void MainWindow::addPathPoint(Point* point){
    qDebug() << "addPathPoint called on point" << point->getName();
    pathCreationWidget->addPathPoint(point);
}

void MainWindow::addPathPoint(PointView* pointView){
    qDebug() << "addPathPoint called on point" << pointView->getPoint()->getName();
    pathCreationWidget->addPathPoint(&(*(pointView->getPoint())));
}

void MainWindow::updatePathPointToPainter(QVector<Point>* pointVector){
    pathPainter->updatePath(*pointVector);
}

void MainWindow::stopPathCreation(){
    for(size_t i = 0; i < pointViews->count(); i++){
        GroupView* groupView = pointViews->getGroups().at(i);
        std::vector<PointView*> pointViews = groupView->getPointViews();
        for(size_t j = 0; j < pointViews.size(); j++){
            pointViews.at(j)->setPixmap(PointView::PixmapType::NORMAL);
        }
    }
}

void MainWindow::hidePathCreationWidget(){
    qDebug() << "hidePathCreationWidget called";
    setGraphicItemsState(GraphicItemState::NO_STATE, true);
    for(size_t i = 0; i < pointViews->count(); i++){
        GroupView* groupView = pointViews->getGroups().at(i);
        std::vector<PointView*> pointViews = groupView->getPointViews();
        for(size_t j = 0; j < pointViews.size(); j++){
            pointViews.at(j)->setPixmap(PointView::PixmapType::NORMAL);
            pointViews.at(j)->setAddedToPath(false);
        }
    }
    pathCreationWidget->resetWidget();
    pathPainter->reset();
    setMessageTop(TEXT_COLOR_NORMAL, "");
}

void MainWindow::saveTmpEditPathPointSlot(void){
    qDebug() << "saveTmpEditPathPointSlot called";
    pathCreationWidget->applySavePathPoint(editedPointView->getPoint()->getPosition().getX(), editedPointView->getPoint()->getPosition().getY());
    editedPointView->setFlag(QGraphicsItem::ItemIsMovable, false);
    setGraphicItemsState(GraphicItemState::CREATING_PATH, false);

    editedPointView = NULL;
}


void MainWindow::moveTmpEditPathPointSlot(void){
    pathCreationWidget->moveEditPathPoint(editedPointView->getPoint()->getPosition().getX(), editedPointView->getPoint()->getPosition().getY());
}

void MainWindow::clearPath(int robotNb){
    if(robots->getRobotsVector().at(robotNb)->getRobot()->isPlayingPath()){
        qDebug() << "pause path on robot before supp " << robotNb << " : " << robots->getRobotsVector().at(robotNb)->getRobot()->getName();
        robots->getRobotsVector().at(robotNb)->getRobot()->setPlayingPath(0);
        bottomLayout->getPlayRobotBtnGroup()->button(robotNb)->setIcon(QIcon(":/icons/play.png"));
    }
    robots->getRobotsVector().at(robotNb)->getRobot()->getPath().clear();
    robots->getRobotsVector().at(robotNb)->getRobot()->setPath(std::vector<std::shared_ptr<PathPoint>>());

    bottomLayout->deletePath(robotNb);
}

void MainWindow::selectHomeEvent(){
    qDebug() << "selectHomeEvent called";
    if(selectedRobotWidget->getScanBtn()->isEnabled()){
        setMessageTop(TEXT_COLOR_INFO, "Click on the map or on a point to select a home for the robot " + selectedRobot->getRobot()->getName());
        selectedRobotWidget->getHomeBtn()->setText("Cancel");
        selectedRobotWidget->disable();
        selectedRobotWidget->getHomeBtn()->setEnabled(true);
        bottomLayout->disable();
        setGraphicItemsState(GraphicItemState::SELECTING_HOME);
        disableMenu();
    } else {
        setMessageTop(TEXT_COLOR_NORMAL,"");
        selectedRobotWidget->getHomeBtn()->setText("Add home");
        selectedRobotWidget->enable();
        bottomLayout->enable();
        setGraphicItemsState(GraphicItemState::NO_STATE);
        enableMenu();
    }
}

void MainWindow::editHomeEvent(){
    qDebug() << "editHomeEvent called";
    if(editSelectedRobotWidget->getNameEdit()->isEnabled()){
        setMessageTop(TEXT_COLOR_INFO, "Click on the map or on a point to select a home for the robot " + selectedRobot->getRobot()->getName());
        editSelectedRobotWidget->getHomeBtn()->setText("Cancel");
        editSelectedRobotWidget->disableAll();
        editSelectedRobotWidget->getHomeBtn()->setEnabled(true);
        bottomLayout->disable();
        setGraphicItemsState(GraphicItemState::EDITING_HOME);
        disableMenu();
    } else {
        setMessageTop(TEXT_COLOR_NORMAL,"");
        if(selectedRobot->getRobot()->getHome() != NULL){
            editSelectedRobotWidget->getHomeBtn()->setText(selectedRobot->getRobot()->getHome()->getName());
        } else {
            editSelectedRobotWidget->getHomeBtn()->setText("Add home");
        }
        editSelectedRobotWidget->enableAll();
        bottomLayout->enable();
        setGraphicItemsState(GraphicItemState::NO_STATE);
        enableMenu();
    }

}

void MainWindow::homeSelected(PointView* pointView, bool temporary){
    qDebug() << "homeSelected called" << pointView->getPoint()->getName();

    int ret = openConfirmMessage("Do you really want to set the point " + pointView->getPoint()->getName() +
                                 + " (" + QString::number(pointView->getPoint()->getPosition().getX(),'f', 1) + ","
                                 + QString::number(pointView->getPoint()->getPosition().getY(),'f', 1) + ") as the home for "
                                 + selectedRobot->getRobot()->getName() +" ?");
    switch(ret){
        case QMessageBox::Cancel :
            pointsLeftWidget->getMinusButton()->setChecked(false);
            if(temporary){
                delete pointView;
            }
        break;
        case QMessageBox::Ok : {

            bool done = false;
            if(temporary){
                qDebug() << "Tmp point";
                if(points.count() > 0 && pointViews->getGroups().size() > 0){
                    pointView->getPoint()->setHome(true, selectedRobot->getRobot()->getName());
                    points.getGroups().at(points.count()-1)->addPoint(pointView->getPoint());
                    XMLParser parserPoints(XML_PATH, mapPixmapItem);
                    parserPoints.save(points);

                    pointViews->setPoints(points);
                    pointViews->getGroups().at(pointViews->getGroups().size()-1)->addPointView(pointView);
                    done = true;
                }
            } else {
                qDebug() << "Permanent point";
                if(pointView->getPoint()->setHome(true, selectedRobot->getRobot()->getName())){
                    XMLParser parserPoints(XML_PATH, mapPixmapItem);
                    parserPoints.save(points);
                    done = true;
                } else {
                    setMessageTop(TEXT_COLOR_DANGER, "Sorry, this point is already a home\nPlease select another");
                }
            }

            pointsLeftWidget->updateGroupButtonGroup(points);

            if(done){
                setMessageTop(TEXT_COLOR_SUCCESS, selectedRobot->getRobot()->getName() + " now has a new home");

                if(selectedRobot->getRobot()->getHome() != NULL){
                    selectedRobot->getRobot()->getHome()->setHome(false, "");
                }

                selectedRobot->getRobot()->setHome(pointView->getPoint());

                selectedRobotWidget->getHomeBtn()->setText("Select a home");
                selectedRobotWidget->enable();
                bottomLayout->enable();
                setGraphicItemsState(GraphicItemState::NO_STATE);
                enableMenu();
                hideAllWidgets();
                selectedRobotWidget->setSelectedRobot(selectedRobot);
                selectedRobotWidget->show();
            }
        }
        break;
        default:
        // should never be here
            qDebug() << " dafuk ?";
        break;
    }
}

void MainWindow::homeEdited(PointView* pointView, bool temporary){
    qDebug() << "homeEdited called" << pointView->getPoint()->getName();

    editSelectedRobotWidget->setHome(pointView, temporary);

    editSelectedRobotWidget->getHomeBtn()->setText(pointView->getPoint()->getName());
    editSelectedRobotWidget->enableAll();
    bottomLayout->enable();
    setGraphicItemsState(GraphicItemState::NO_STATE);
    enableMenu();
}

void MainWindow::showHome(){
    qDebug() << "showHome called" << (selectedRobot->getRobot()->getHome()==NULL);
    for(size_t i = 0; i < pointViews->getGroups().size(); i++){
        GroupView* groupView = pointViews->getGroups().at(i);
        for(size_t j = 0; j < groupView->getPointViews().size(); j++){
            groupView->getPointViews().at(j)->QGraphicsPixmapItem::setPixmap(QPixmap(PIXMAP_NORMAL));
        }
    }
    if(selectedRobot->getRobot()->getHome() != NULL){
        PointView* pointView = pointViews->getPointViewFromPoint(*(selectedRobot->getRobot()->getHome()));
        pointView->setPixmap(PointView::PixmapType::NORMAL);
    }
}

void MainWindow::hideHome(void){
    for(size_t i = 0; i < pointViews->getGroups().size(); i++){
        GroupView* groupView = pointViews->getGroups().at(i);
        for(size_t j = 0; j < groupView->getPointViews().size(); j++){
            groupView->getPointViews().at(j)->QGraphicsPixmapItem::setPixmap(QPixmap(PIXMAP_NORMAL));
        }
    }
}

void MainWindow::goHomeBtnEvent(){
    qDebug() << "go home robot " << selectedRobot->getRobot()->getName() << (selectedRobot->getRobot()->getHome() == NULL);
    float oldPosX = selectedRobot->getRobot()->getHome()->getPosition().getX();
    float oldPosY = selectedRobot->getRobot()->getHome()->getPosition().getY();
    qDebug() << "Go to next point :" << oldPosX << oldPosY;
    qDebug() << "ok1" << (float) oldPosX;
    qDebug() << "ok2" << (float) (oldPosX - ROBOT_WIDTH);
    qDebug() << "ok3" << (float) ((oldPosX - ROBOT_WIDTH) * map->getResolution());
    qDebug() << "ok4" << (float) ((oldPosX - ROBOT_WIDTH) * map->getResolution() + map->getOrigin().getX());

    float newPosX = (oldPosX - ROBOT_WIDTH) * map->getResolution() + map->getOrigin().getX();
    float newPosY = (-oldPosY + map->getHeight() - ROBOT_WIDTH/2) * map->getResolution() + map->getOrigin().getY();
    qDebug() << "Go to next point :" << newPosX << newPosY;
    int waitTime = -1;

    /// if the command is succesfully sent to the robot, we apply the change
    if(selectedRobot->getRobot()->sendCommand(QString("c ") + QString::number(newPosX) + " "  + QString::number(newPosY) + " "  + QString::number(waitTime))){
        qDebug() << "Going to home";
    }
}

/**********************************************************************************************************************************/

//                                          MAPS

/**********************************************************************************************************************************/

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
            }
        }
        break;
        default:
        // should never be here
            qDebug() << " dafuk ?";
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
    //lastWidget = mapLeftWidget;
}

/**********************************************************************************************************************************/

//                                          MENUS

/**********************************************************************************************************************************/


void MainWindow::initializeLeftMenu(){
    //lastWidget = leftMenu->getLastWidget();
    lastWidget =  QList<QWidget*>();
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

void MainWindow::initializeBottomPanel(){
    bottomLayout = new BottomLayout(this, robots);
    rightLayout->addWidget(bottomLayout);
}

void MainWindow::disableMenu(){
    topLayout->disable();
}

void MainWindow::enableMenu(){
    topLayout->enable();
}

void MainWindow::setMessageTop(QString msgType, QString msg){
    topLayout->setLabel(msgType, msg);
}

void MainWindow::closeSlot(){
    resetFocus();
    leftMenu->hide();
   // leftMenu->hideBackButton();

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
    XMLParser pParser(":/xml/points.xml", mapPixmapItem);
    pParser.readPoints(points);
    /// displays the points that need to be displayed on the map
    pointViews = new PointsView(points, mapPixmapItem);
    for(size_t j = 0; j < pointViews->count(); j++){
        for(size_t k = 0; k < pointViews->getGroups().at(j)->getPointViews().size(); k++){
            /// establish the right connections in order to use the points to create paths and home points
            connect(pointViews->getGroups().at(j)->getPointViews().at(k),
                    SIGNAL(addPointPath(PointView*)), this,
                    SLOT(addPathPoint(PointView*)));
            connect(pointViews->getGroups().at(j)->getPointViews().at(k), SIGNAL(homeSelected(PointView*, bool)), this, SLOT(homeSelected(PointView*, bool)));
            connect(pointViews->getGroups().at(j)->getPointViews().at(k), SIGNAL(homeEdited(PointView*, bool)), this, SLOT(homeEdited(PointView*, bool)));
        }
    }
}

/**
 * @brief MainWindow::setSelectedPoint
 * @param pointView
 * @param isTemporary
 * set the selected point, could be a temporary point or a point that already exists and that might be edited
 */
void MainWindow::setSelectedPoint(PointView* pointView, bool isTemporary){
    /// we are not modifying an existing point
    if(!leftMenu->getDisplaySelectedPoint()->getEditButton()->isChecked()){
        leftMenu->show();
        selectedPoint = pointView;
        selectedPoint->setState(GraphicItemState::EDITING_PERM);
        hideAllWidgets();
        editSelectedPointWidget->setSelectedPoint(selectedPoint, isTemporary);
        editSelectedPointWidget->show();
        leftMenu->getDisplaySelectedPoint()->hide();
    } else {
        /// on the left we display the position of the temporary point as the user moves it around but we don't make any modifications on the model yet
        leftMenu->getDisplaySelectedPoint()->getXLabel()->setText(QString::number(mapPixmapItem->getTmpPointView()->getPoint()->getPosition().getX()));
        leftMenu->getDisplaySelectedPoint()->getYLabel()->setText(QString::number(mapPixmapItem->getTmpPointView()->getPoint()->getPosition().getY()));
    }
}

/**
 * @brief MainWindow::pointBtnEvent
 * called when the back button is clicked
 */
void MainWindow::pointBtnEvent(void){
    switchFocus("Points",pointsLeftWidget);
    qDebug() << "pointBtnEvent called ";
    /// we uncheck all buttons from all menus
    pointsLeftWidget->getGroupButtonGroup()->uncheck();
    leftMenu->getDisplaySelectedGroup()->uncheck();
    pointsLeftWidget->getEyeButton()->setChecked(false);
    /// called when the back button is clicked and we came from the group menu
    if(leftMenu->getDisplaySelectedPoint()->getOrigin() == DisplaySelectedPoint::GROUP_MENU){
        hideAllWidgets();
        leftMenu->getDisplaySelectedGroup()->getEditButton()->setChecked(false);
        leftMenu->getDisplaySelectedPoint()->setOrigin(DisplaySelectedPoint::POINTS_MENU);
        qDebug() << "dans pointBtn event with origin group menu";
        leftMenu->getDisplaySelectedGroup()->show();
    }
    /// otherwise we know that we can display the points menu because the case where we displayed the point information
    /// from the map has already been taken care of at the source
    else {
        hideAllWidgets();
        leftMenu->getDisplaySelectedPoint()->setOrigin(DisplaySelectedPoint::POINTS_MENU);
        pointsLeftWidget->show();
        pointsLeftWidget->getGroupButtonGroup()->show();
        //lastWidget = pointsLeftWidget;
    }
}


/**
 * @brief MainWindow::backGroupBtnEvent
 * called when the back button of the first points menu is clicked (to go back to the general menu)
 */
void MainWindow::backGroupBtnEvent(){
    qDebug() << "backGroupBtnEvent called";
    /// uncheck the buttons in the first menu
    pointsLeftWidget->getGroupButtonGroup()->uncheck();
    /// uncheck the other buttons
    pointsLeftWidget->getPlusButton()->setChecked(false);
    pointsLeftWidget->getMinusButton()->setChecked(false);
    pointsLeftWidget->getEditButton()->setChecked(false);
    pointsLeftWidget->getEyeButton()->setChecked(false);
    pointsLeftWidget->getMapButton()->setChecked(false);

    /// we hide those in case the previous button clicked was the plus button
    pointsLeftWidget->getGroupNameEdit()->hide();
    pointsLeftWidget->getGroupNameLabel()->hide();

    pointsLeftWidget->hide();
    leftMenuWidget->show();
}

void MainWindow::plusGroupBtnEvent(){
    qDebug() << "plusGroupBtnEvent called";
    /// uncheck the other buttons
    pointsLeftWidget->getPlusButton()->setChecked(false);
    pointsLeftWidget->getMinusButton()->setChecked(false);
    pointsLeftWidget->getEditButton()->setChecked(false);
    pointsLeftWidget->getEyeButton()->setChecked(false);
    pointsLeftWidget->getMapButton()->setChecked(false);

    /// here we allow a user to create a new group
    pointsLeftWidget->getGroupNameEdit()->show();
    pointsLeftWidget->getGroupNameLabel()->show();
}

/**
 * @brief MainWindow::minusGroupBtnEvent
 * called in the first points menu to either remove a group or a point which belongs to the default group
 */
void MainWindow::minusGroupBtnEvent(){
    qDebug() << "minusGroupBtnEvent called";

    /// uncheck the other buttons
    pointsLeftWidget->getPlusButton()->setChecked(false);
    pointsLeftWidget->getEditButton()->setChecked(false);
    pointsLeftWidget->getEyeButton()->setChecked(false);
    pointsLeftWidget->getMapButton()->setChecked(false);

    /// we hide those in case the previous button clicked was the plus button
    pointsLeftWidget->getGroupNameEdit()->hide();
    pointsLeftWidget->getGroupNameLabel()->hide();

    int checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedId();

    /// we have to delete a group
    if(checkedId > -1 && checkedId < points.count()-1)
        askForDeleteGroupConfirmation(checkedId);

    /// we have to delete a point
    else if(checkedId >= points.count()-1)
        askForDeleteDefaultGroupPointConfirmation(checkedId-points.count()+1);
}

/**
 * @brief MainWindow::editPointButtonEvent
 * @param checked
 * called to edit an existing point
 */
void MainWindow::editPointButtonEvent(bool checked){

    qDebug() << "editPointButtonEvent called";

    /// update buttons enable attribute and tool tips
    leftMenu->getDisplaySelectedPoint()->getMapButton()->setChecked(true);
    leftMenu->getDisplaySelectedPoint()->getMinusButton()->setEnabled(false);
    leftMenu->getDisplaySelectedPoint()->getMinusButton()->setToolTip("");
    leftMenu->getDisplaySelectedPoint()->getMapButton()->setEnabled(false);
    leftMenu->getDisplaySelectedPoint()->getMapButton()->setToolTip("");
    leftMenu->getDisplaySelectedPoint()->getBackButton()->setEnabled(false);

    /// hide the temporary point on the map
    mapPixmapItem->getTmpPointView()->hide();
    /// change the color of the pointview that's selected on the map
    leftMenu->getDisplaySelectedPoint()->getPointView()->setPixmap(PointView::PixmapType::HOVER);
    leftMenu->getDisplaySelectedPoint()->getPointView()->show();
    /// updates the model
    leftMenu->getDisplaySelectedPoint()->getPoint()->setDisplayed(true);

    /// updates the file
    XMLParser parser(XML_PATH, mapPixmapItem);
    parser.save(points);

    /// uncheck the other buttons
    pointsLeftWidget->getPlusButton()->setChecked(false);
    pointsLeftWidget->getMinusButton()->setChecked(false);
    pointsLeftWidget->getEyeButton()->setChecked(false);
    pointsLeftWidget->getMapButton()->setChecked(false);

    /// this way we force the user to either click save or cancel
    leftMenu->getDisplaySelectedPoint()->getEditButton()->setEnabled(false);
    leftMenu->getDisplaySelectedPoint()->getEditButton()->setToolTip("You can choose to save or discard your modifications by clicking the save (Enter) and cancel button respectively");

    /// we show the save button and the cancel button
    leftMenu->getDisplaySelectedPoint()->getCancelButton()->show();
    leftMenu->getDisplaySelectedPoint()->getSaveButton()->show();

    /// makes it obvious what the user has to do to change the name of his point
    leftMenu->getDisplaySelectedPoint()->getNameEdit()->setAutoFillBackground(false);
    leftMenu->getDisplaySelectedPoint()->getNameEdit()->setFrame(true);

    /// if the point is a home point any modification of its name is forbidden
    if(!leftMenu->getDisplaySelectedPoint()->getPoint()->isHome())
        leftMenu->getDisplaySelectedPoint()->getNameEdit()->setReadOnly(false);

    /// sets the state of the map and the other widgets to prevent other concurrent actions
    setGraphicItemsState(GraphicItemState::NO_EVENT, false);
    mapPixmapItem->setState(GraphicItemState::EDITING_PERM);

    /// sets the state of the point of the map to make it draggable
    pointViews->getPointViewFromPoint(*(leftMenu->getDisplaySelectedPoint()->getPoint()))->setState(GraphicItemState::EDITING_PERM);
    pointViews->getPointViewFromPoint(*(leftMenu->getDisplaySelectedPoint()->getPoint()))->setFlag(QGraphicsItem::ItemIsMovable, true);
}

/**
 * @brief MainWindow::editGroupBtnEvent
 * @param checked
 * called when the user wants to edit a point from the first points menu
 */
void MainWindow::editGroupBtnEvent(bool checked){
    qDebug() << "editPointBtnEvent called";
    /// uncheck the other buttons
    pointsLeftWidget->getPlusButton()->setChecked(false);
    pointsLeftWidget->getMinusButton()->setChecked(false);
    pointsLeftWidget->getEyeButton()->setChecked(false);
    pointsLeftWidget->getMapButton()->setChecked(false);

    /// we hide those in case the previous button clicked was the plus button
    pointsLeftWidget->getGroupNameEdit()->hide();
    pointsLeftWidget->getGroupNameLabel()->hide();

    int checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedId();
    /// it's an isolated point
    if(checkedId != -1 && checkedId >= points.count()-1){
        std::shared_ptr<Point> point = points.findPoint(points.count()-1, checkedId-points.count()+1);
        /// retrieves the pointview associated to the point on the map and displays it if it was not already the case
        PointView* pointView = pointViews->getPointViewFromPoint(*point);
        pointView->show();
        /// must display the tick icon in the pointsLeftWidget
        pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/tick.png"));
        if(pointView)
            leftMenu->getDisplaySelectedPoint()->setPointView(pointView);
        else
            qDebug() << "THere is no point view associated with those indexes";
        /// displays the information relative the the point
        leftMenu->getDisplaySelectedPoint()->displayPointInfo();
        editPointButtonEvent(checked);
        pointsLeftWidget->hide();
        /// disables the back button to prevent problems, a user has to discard or save his modifications before he can start navigatin the menu again, also prevents false manipulations
        leftMenu->getDisplaySelectedPoint()->getBackButton()->setEnabled(false);
        leftMenu->getDisplaySelectedPoint()->getBackButton()->setToolTip("Please save or discard your modifications before navigating the menu again.");
        leftMenu->getDisplaySelectedPoint()->show();
    }
}

void MainWindow::selectPointBtnEvent(){
    qDebug() << "selectPointBtnEvent called";
}

void MainWindow::switchFocus(QString name, QWidget* widget)
{
    if(currentWidget != NULL)
    {
        qDebug() << "currentWidgetNOTNULL";
        lastWidget.append(currentWidget);
        lastName.append(currentName);
    }


    qDebug() << "__________________";


    currentWidget=widget;
    currentName=name;
    if(lastWidget.size()!=0)
    {
     leftMenu->showBackButton(&lastName.last());
    }
    else
    {
        leftMenu->hideBackButton();
    }
    for(int i=0;i<lastName.size();i++)
    {
        qDebug() << lastName.at(i);
    }
}
void MainWindow::resetFocus()
{
    currentName = "";
    currentWidget = NULL;
    lastName = QList<QString>();
    lastWidget = QList<QWidget*>();
}

void MainWindow::updateView()
{
    if(currentWidget == NULL)
         leftMenu->hideBackButton();
    else
        leftMenu->showBackButton(&currentName);
}

void MainWindow::openLeftMenu(){
    qDebug() << "openLeftMenu called";
    resetFocus();
    if(leftMenu->isHidden()){

        hideAllWidgets();
        leftMenuWidget->show();
        leftMenu->show();
        switchFocus("Menu",leftMenuWidget);

    } else {
        /// we reset the origin of the point information menu in order to display the buttons to go back in the further menus
        leftMenu->getDisplaySelectedPoint()->setOrigin(DisplaySelectedPoint::POINTS_MENU);
        leftMenu->getDisplaySelectedPoint()->hide();
        if(leftMenuWidget->isHidden()){
            robotsLeftWidget->setEditBtnStatus(false);
            robotsLeftWidget->setCheckBtnStatus(false);

            hideAllWidgets();
            leftMenuWidget->show();
            leftMenu->show();
            switchFocus("Menu",leftMenuWidget);

        } else {
                closeSlot();
        }
    }
}

/**
 * @brief MainWindow::backSelecPointBtnEvent
 * called when the back button is clicked on the point information menu
 */
void MainWindow::backSelecPointBtnEvent(){
    qDebug() << "backSelecPointBtnEvent called";
    selectedPointWidget->hide();
    /// if we come from a menu we display it again

    if(lastWidget.last() != NULL){
        lastWidget.last()->show();


    } else {
        /// we come from the map we don't display anything
        leftMenu->hide();
    }
}

void MainWindow::minusSelecPointBtnEvent(){
    qDebug() << "minusSelecPointBtnEvent called";
}

void MainWindow::editSelecPointBtnEvent(){
    qDebug() << "editSelecPointBtnEvent called";
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

/**
 * @brief MainWindow::pointSavedEvent
 * @param index
 * @param x
 * @param y
 * @param name
 * called when a permanent point is created from a temporary one using enter or the save button
 */
void MainWindow::pointSavedEvent(int index, double x, double y, QString name){

    qDebug() << "pointSavedEvent called";

    /// resets the status of the plus button
    editSelectedPointWidget->getPlusButton()->setEnabled(true);
    /// hides widgets relative to the choice of a group
    editSelectedPointWidget->hideGroupLayout();

    std::shared_ptr<Point> newPoint = std::shared_ptr<Point> (new Point(name, x, y, true, true));

    /// default group
    if(index == 0)
        points.getGroups().at(points.count()-1)->addPoint(newPoint);
    else
        points.getGroups().at(index-1)->addPoint(newPoint);

    /// creates a new point on the map
    PointView* newPointView = new PointView(newPoint, mapPixmapItem);
    mapPixmapItem->addPointView(newPointView);

    /// saves it to the file
    XMLParser parser(XML_PATH, mapPixmapItem);
    parser.save(points);

    /// updates the menu
    pointsLeftWidget->updateGroupButtonGroup(points);

    /// hides the temporary point so that they don't superimpose which is confusing when hiding / showing the newly created point
    mapPixmapItem->getTmpPointView()->hide();
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

/**
 * @brief MainWindow::askForDeleteDefaultGroupPointConfirmation
 * @param index
 * Called when a user wants to remove a point that belongs to the default group
 * the index given is the index of the point within its group
 */
void MainWindow::askForDeleteDefaultGroupPointConfirmation(int index){
    int ret = openConfirmMessage("Do you really want to remove this point ?");
    switch(ret){
        case QMessageBox::Cancel :
            pointsLeftWidget->getMinusButton()->setChecked(false);
        break;
        case QMessageBox::Ok : {
        /// we first check that our point is not the home of a robot
            std::shared_ptr<Point> point = points.getGroups().at(points.count()-1)->getPoints().at(index);
            if(!point->isHome()){
                qDebug() << "Go ahead and remove me I am not a home point anyway";
                pointsLeftWidget->getMinusButton()->setChecked(false);
                /// updates the model
                points.getGroups().at(points.count()-1)->removePoint(index);
                /// save changes in the file
                XMLParser parserPoints(XML_PATH, mapPixmapItem);
                parserPoints.save(points);
                /// updates the menu
                pointsLeftWidget->getGroupButtonGroup()->update(points);
                /// need to remove the point from the map
                pointViews->getPointViewFromPoint(*point)->hide();
            } else {
                /// this is in fact the home point of a robot, we prompt a customized message to the end user
                RobotView* robot = robots->findRobotUsingHome(point->getName());
                openInterdictionOfPointRemovalMessage(point->getName(), robot->getRobot()->getName());
                qDebug() << "Sorry this point is the home of a robot and therefore cannot be removed";
            }
        }
        break;
        default:
        /// should never be here
            qDebug() << " dafuk ?";
        break;
    }
}

/**
 * @brief MainWindow::askForDeletePointConfirmation
 * @param index
 * Called when a user wants to remove a point that belongs to any group but the default one
 * the index given is the index of the point within its group
 */
void MainWindow::askForDeletePointConfirmation(int index){
    qDebug() << "askfordeletepointconfirmation event called";
    int ret = openConfirmMessage("Do you really want to remove this point ?");
    switch(ret){
        case QMessageBox::No :
            qDebug() << "clicked no";
        break;
        case QMessageBox::Ok : {
        /// we first check that our point is not the home of a robot
            std::shared_ptr<Point> point = points.getGroups().at(pointsLeftWidget->getIndexLastGroupClicked())->getPoints().at(index);
            if(!point->isHome()){
                qDebug() << "Go ahead and remove me I am not a home point anyway";
                /// need to remove the point from the map
                pointViews->getPointViewFromPoint(*point)->hide();
                /// updates the model
                points.getGroups().at(pointsLeftWidget->getIndexLastGroupClicked())->removePoint(index);
                /// updates the group menu
                qDebug() << "group set is " << pointsLeftWidget->getIndexLastGroupClicked();
                leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->setGroup(points, pointsLeftWidget->getIndexLastGroupClicked());
                /// save the changes to the file
                XMLParser parserPoints(XML_PATH, mapPixmapItem);
                parserPoints.save(points);
                leftMenu->getDisplaySelectedPoint()->getMinusButton()->setChecked(false);
                /// makes the buttons checkable again
                leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->setCheckable(true);
            } else {
                /// this is in fact the home point of a robot, we prompt a customized message to the end user
                RobotView* robot = robots->findRobotUsingHome(point->getName());
                qDebug() << robot->getRobot()->getName();
                openInterdictionOfPointRemovalMessage(point->getName(), robot->getRobot()->getName());
                qDebug() << "Sorry this point is the home of a robot and therefore cannot be removed";
            }
        }
        break;
        default:
        /// should never be here
            qDebug() << " dafuk ?";
        break;
    }
}

/**
 * @brief MainWindow::askForDeleteGroupConfirmation
 * @param index
 * Called when a user wants to remove a whole group of points
 * the index given is the index of the group within the Points object
 */
void MainWindow::askForDeleteGroupConfirmation(int index){
    int ret = openConfirmMessage("Do you really want to remove this group ? All the points in this group would also be removed.");
    switch(ret){
        case QMessageBox::Cancel :
            qDebug() << "clicked no";
            pointsLeftWidget->getMinusButton()->setChecked(false);
        break;
        case QMessageBox::Ok : {
        /// we have to check that none of the points is the home of a robot
            std::shared_ptr<Point> homePoint = points.getGroups().at(index)->containsHomePoint();
            if(!homePoint){
                /// removes all the points of the group on the map
                for(int i = 0; i < points.getGroups().at(index)->count(); i++){
                    pointViews->getPointViewFromPoint(*points.getGroups().at(index)->getPoints().at(i))->hide();
                }
                /// removes the group from the model
                points.removeGroup(index);
                /// updates the file
                XMLParser parserPoints(XML_PATH, mapPixmapItem);
                parserPoints.save(points);
                /// updates the menu
                pointsLeftWidget->getGroupButtonGroup()->update(points);
                pointsLeftWidget->getMinusButton()->setChecked(false);
            } else {
                /// this group contains the home point of a robot and cannot be removed, we prompt the end user with a customized message to explain which robot has its home point in the group
                RobotView* robot = robots->findRobotUsingHome(homePoint->getName());
                QMessageBox msgBox;
                msgBox.setText("This group contains the point : " + homePoint->getName() + " which is the home point of the robot " + robot->getRobot()->getName() +
                               ". If you want to remove it you first have to indicate a new home point for this robot.");
                msgBox.setIcon(QMessageBox::Critical);
                msgBox.setStandardButtons(QMessageBox::Ok);
                msgBox.setInformativeText("To modify the home point of a robot you can either click on the menu > Robots, choose a robot and Add home or simply click a robot on the map and Add home");
                msgBox.exec();
                qDebug() << "Sorry this point is the home of a robot and therefore cannot be removed";
            }
        }
        break;
        default:
            /// should never be here
            qDebug() << " dafuk ?";
        break;
    }
}

void MainWindow::displayPointEvent(PointView* pointView){
    leftMenu->getDisplaySelectedPoint()->getMapButton()->setChecked(true);
    leftMenu->getDisplaySelectedPoint()->setOrigin(DisplaySelectedPoint::MAP);
    leftMenu->getDisplaySelectedPoint()->setPointView(pointView);
    pointView->setState(GraphicItemState::NO_STATE);
    qDebug() << leftMenu->getDisplaySelectedPoint()->getPointView()->getPoint()->getName() <<
                                                                                  QString::number(leftMenu->getDisplaySelectedPoint()->getPointView()->getPoint()->getPosition().getX()) <<
                                                                                  QString::number(leftMenu->getDisplaySelectedPoint()->getPointView()->getPoint()->getPosition().getY());
    leftMenu->getDisplaySelectedPoint()->displayPointInfo();
    hideAllWidgets();
    if(leftMenu->isHidden()){
        leftMenu->show();
    }
    leftMenu->getDisplaySelectedPoint()->show();
}

void MainWindow::modifyGroupEvent(int groupIndex){
    /*
    qDebug() << groupIndex;
    if(groupIndex < points.count()-1)
        qDebug() << "modifyGroupEvent";
        */
    /// uncheck the other buttons
    pointsLeftWidget->getPlusButton()->setChecked(false);
    pointsLeftWidget->getMinusButton()->setChecked(false);
    pointsLeftWidget->getEditButton()->setChecked(false);
    pointsLeftWidget->getEyeButton()->setChecked(false);
    pointsLeftWidget->getMapButton()->setChecked(false);
}

void MainWindow::displayGroupEvent(int index, bool display){
    qDebug() << "displaygroupevent : group " << index << " ";
    /// uncheck the other buttons
    pointsLeftWidget->getPlusButton()->setChecked(false);
    pointsLeftWidget->getMinusButton()->setChecked(false);
    pointsLeftWidget->getEditButton()->setChecked(false);
    pointsLeftWidget->getEyeButton()->setChecked(false);

    if(pointsLeftWidget->getMapButton()->isChecked()){
        if(display){
            if(index < points.count()-1){
                qDebug() << " i have to display a whole group ";
                for(int i = 0; i < points.getGroups().at(index)->getPoints().size(); i++){
                    std::shared_ptr<Point> currentPoint = points.getGroups().at(index)->getPoints().at(i);
                    currentPoint->setDisplayed(true);
                    pointViews->getPointViewFromPoint(*currentPoint)->show();
                    XMLParser parserPoints(XML_PATH, mapPixmapItem);
                    parserPoints.save(points);
                }
            } else {
                std::shared_ptr<Point> currentPoint = points.getGroups().at(points.count()-1)->getPoints().at(index-points.count()+1);
                currentPoint->setDisplayed(true);
                pointViews->getPointViewFromPoint(*currentPoint)->show();
                XMLParser parserPoints(XML_PATH, mapPixmapItem);
                parserPoints.save(points);

            }
        } else {
            if(index < points.count()-1){
                qDebug() << " i have to stop displaying a whole group ";
                for(int i = 0; i < points.getGroups().at(index)->getPoints().size(); i++){
                    std::shared_ptr<Point> currentPoint = points.getGroups().at(index)->getPoints().at(i);
                    currentPoint->setDisplayed(false);
                    pointViews->getPointViewFromPoint(*currentPoint)->hide();
                    XMLParser parserPoints(XML_PATH, mapPixmapItem);
                    parserPoints.save(points);
                }
            } else {
                qDebug() << " i have to stop displaying a single point which index is " << index-points.count()+1;
                std::shared_ptr<Point> currentPoint = points.getGroups().at(points.count()-1)->getPoints().at(index-points.count()+1);
                currentPoint->setDisplayed(false);
                pointViews->getPointViewFromPoint(*currentPoint)->hide();
                XMLParser parserPoints(XML_PATH, mapPixmapItem);
                parserPoints.save(points);
            }
        }
    }
}

void MainWindow::displayGroupMapEvent(void){
    /// uncheck the other buttons
    pointsLeftWidget->getPlusButton()->setChecked(false);
    pointsLeftWidget->getMinusButton()->setChecked(false);
    pointsLeftWidget->getEditButton()->setChecked(false);
    pointsLeftWidget->getEyeButton()->setChecked(false);

    /// we hide those in case the previous button clicked was the plus button
    pointsLeftWidget->getGroupNameEdit()->hide();
    pointsLeftWidget->getGroupNameLabel()->hide();

    qDebug() << "displaying groups by clicking on the map button";
    int checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedId();
    /// we display groups
    if(checkedId > -1 && checkedId < points.count()-1){
        /// the group was displayed, we now have to hide it (all its points)
        if(points.getGroups().at(checkedId)->isDisplayed()){
            pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/folder.png"));
            for(int i = 0; i < points.getGroups().at(checkedId)->count(); i++){
                std::shared_ptr<Point> point = points.getGroups().at(checkedId)->getPoints()[i];
                PointView* pointView = pointViews->getPointViewFromPoint(*point);
                point->setDisplayed(false);
                pointView->hide();
                /// update the file
                XMLParser parserPoints(XML_PATH, mapPixmapItem);
                parserPoints.save(points);
            }
        } else {
            /// the group must now be displayed
            pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/folder_tick.png"));
            for(int i = 0; i < points.getGroups().at(checkedId)->count(); i++){
                std::shared_ptr<Point> point = points.getGroups().at(checkedId)->getPoints()[i];
                PointView* pointView = pointViews->getPointViewFromPoint(*point);
                point->setDisplayed(true);
                pointView->show();
                /// update the file
                XMLParser parserPoints(XML_PATH, mapPixmapItem);
                parserPoints.save(points);
            }
        }
    }
    /// we display isolated points
    else if(checkedId >= points.count()-1){
        std::shared_ptr<Point> point = points.getGroups().at(points.count()-1)->getPoints().at(checkedId-points.count()+1);
        /// if the point is displayed we hide it
        if(point->isDisplayed()){
            pointViews->getPointViewFromPoint(*point)->hide();
            point->setDisplayed(false);
            /// update the file
            XMLParser parserPoints(XML_PATH, mapPixmapItem);
            parserPoints.save(points);
            /// we remove the tick icon
            pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon());
        } else {
            /// the point was not displayed, we display it
            pointViews->getPointViewFromPoint(*point)->show();
            point->setDisplayed(true);
            XMLParser parserPoints(XML_PATH, mapPixmapItem);
            parserPoints.save(points);
            /// we add the tick icon
            pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/tick.png"));
        }
    }
}

void MainWindow::displayPointMapEvent(){
    qDebug() << "displaypoint map event";
    PointView* pointView = leftMenu->getDisplaySelectedPoint()->getPointView();
    std::pair<int, int> pointIndexes = points.findPointIndexes(pointView->getPoint()->getName());
    qDebug() << "indexes are " << pointIndexes.first << pointIndexes.second;

    if(pointView->getPoint()){
        if(pointView->getPoint()->isDisplayed()){
            qDebug() << " I was displayed, but it's over";
            leftMenu->getDisplaySelectedPoint()->getMapButton()->setToolTip("Click to display this point");
            pointView->getPoint()->setDisplayed(false);
            pointView->hide();
            /// update the file
            XMLParser parserPoints(XML_PATH, mapPixmapItem);
            parserPoints.save(points);
            /// we update the group menu
            /// it's a point that belongs to a group
            if(pointIndexes.first < points.count()-1){
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(pointIndexes.first)->setIcon(QIcon(":/icons/folder.png"));
                leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->button(pointIndexes.second)->setIcon(QIcon());
            }
            /// it's an isolated point
            else
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(points.count()+pointIndexes.second-1)->setIcon(QIcon());

        } else {
            qDebug() << " Now I have returned to be displayed";
            leftMenu->getDisplaySelectedPoint()->getMapButton()->setToolTip("Click to hide this point");
            pointView->getPoint()->setDisplayed(true);
            pointView->show();
            /// update the file
            XMLParser parserPoints(XML_PATH, mapPixmapItem);
            parserPoints.save(points);
            /// we update the groups menu
            /// it's a point that belongs to a group
            if(pointIndexes.first < points.count()-1){
                leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->button(pointIndexes.second)->setIcon(QIcon(":/icons/tick.png"));
                /// we check whether or not the entire group is displayed and update the points left widget accordingly by adding a tick Icon or not
                if(points.getGroups().at(pointsLeftWidget->getIndexLastGroupClicked())->isDisplayed())
                    pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(pointIndexes.first)->setIcon(QIcon(":/icons/folder_tick.png"));
            }
            /// it's an isolated point
            else
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(points.count()+pointIndexes.second-1)->setIcon(QIcon(":/icons/tick.png"));
        }
    } else {
        qDebug() << "wtf am I manipulating a NULL pointer ?";
    }
}

/**
 * @brief MainWindow::removeGroupEvent
 * @param groupIndex
 * called when a user tries to either remove a group or a point from the menu that lists the groups
 */
void MainWindow::removeGroupEvent(const int groupIndex){
    if(pointsLeftWidget->getMinusButton()->isChecked()){
        /// retrieves the id of the checked button within the group of buttons
        int checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedId();

        /// we have to delete a group
        if(checkedId > -1 && checkedId < points.count()-1)
            askForDeleteGroupConfirmation(checkedId);

        /// we have to delete a point
        else if(checkedId >= points.count()-1)
            askForDeleteDefaultGroupPointConfirmation(checkedId-points.count()+1);
    }
}

/**
 * @brief MainWindow::displayPointsInGroup
 * called when a user clicks the "eye" button in the Points menu
 */
void MainWindow::displayPointsInGroup(void){
    qDebug() << "display points in group";
    /// uncheck the other buttons
    pointsLeftWidget->getPlusButton()->setChecked(false);
    pointsLeftWidget->getMinusButton()->setChecked(false);
    pointsLeftWidget->getEditButton()->setChecked(false);
    pointsLeftWidget->getMapButton()->setChecked(false);

    /// we hide those in case the previous button clicked was the plus button
    pointsLeftWidget->getGroupNameEdit()->hide();
    pointsLeftWidget->getGroupNameLabel()->hide();

    /// retrieves the id of the checked button within the group of buttons
    int checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedId();

    /// it's a group
    if(checkedId != -1 && checkedId < points.count()-1){
       pointsLeftWidget->setIndexLastGroupClicked(checkedId);
       pointsLeftWidget->getEyeButton()->setChecked(false);
       pointsLeftWidget->hide();
       /// before we display the group of points, we make sure that the graphical object is consistent with the model
       DisplaySelectedGroup* selectedGroup = leftMenu->getDisplaySelectedGroup();
       leftMenu->updateGroupDisplayed(points, checkedId);
       selectedGroup->getPointButtonGroup()->setCheckable(true);
       selectedGroup->show();
       selectedGroup->setName(points.getGroups().at(checkedId)->getName());
       switchFocus(points.getGroups().at(checkedId)->getName(),selectedGroup);
    }
    /// it's an isolated point
    else if(checkedId >= points.count()-1){
        DisplaySelectedPoint* selectedPoint = leftMenu->getDisplaySelectedPoint();
        PointView* pointView = pointViews->getPointViewFromPoint(*(points.getGroups().at(points.count()-1)->getPoints().at(checkedId+1-points.count())));
        selectedPoint->setPointView(pointView);
        selectedPoint->displayPointInfo();
        selectedPoint->show();
        if(pointView->getPoint()->isDisplayed())
            selectedPoint->getMapButton()->setChecked(true);
        else
            selectedPoint->getMapButton()->setChecked(false);
        switchFocus("Point",selectedPoint);
        pointsLeftWidget->getEyeButton()->setChecked(false);
        pointsLeftWidget->hide();
    }
}

/**
 * @brief MainWindow::removePointFromInformationMenu
 * called when a user clicks a point on the map and then tries to remove it clicking the "minus" button
 */
void MainWindow::removePointFromInformationMenu(void){
    qDebug() << "removepointfrominformationmenu event called";
    /// uncheck the minus button
    leftMenu->getDisplaySelectedPoint()->getMinusButton()->setChecked(false);
    int ret = openConfirmMessage("Are you sure you want to remove this point ?");
    switch(ret){
        case QMessageBox::Cancel :
            qDebug() << "clicked no";
        break;
        case QMessageBox::Ok : {
        /// first we check that this point is not a home
            std::shared_ptr<Point> point = leftMenu->getDisplaySelectedPoint()->getPoint();
            if(!point->isHome()){
                QString pointName = leftMenu->getDisplaySelectedPoint()->getPointName();
                /// holds the index of the group and the index of a particular point in this group within <points>
                std::pair<int, int> pointIndexes = points.findPointIndexes(pointName);
                if(pointIndexes.first != -1){
                    /// need to remove the point from the map
                    pointViews->getPointViewFromPoint(*point)->hide();
                    /// updates the model
                    points.getGroups().at(pointIndexes.first)->removePoint(pointIndexes.second);
                    /// updates the file containing containing points info
                    XMLParser parserPoints(XML_PATH, mapPixmapItem);
                    parserPoints.save(points);
                    /// updates the group menu and the list of points
                    pointsLeftWidget->getGroupButtonGroup()->update(points);
                    /// closes the window
                    leftMenu->hide();
                    /// depending on how we got there we display a menu or not
                    if(leftMenu->getDisplaySelectedPoint()->getOrigin() == DisplaySelectedPoint::GROUP_MENU)
                        leftMenu->getDisplaySelectedGroup()->show();
                    else if(leftMenu->getDisplaySelectedPoint()->getOrigin() == DisplaySelectedPoint::POINTS_MENU)
                        pointsLeftWidget->show();

                    leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->setGroup(points, pointsLeftWidget->getIndexLastGroupClicked());
                } else {
                    qDebug() << "could not find this point";
                }
            } else {
                /// this point is actually the home point of a robot and therefore cannot be removed
                RobotView* robot = robots->findRobotUsingHome(point->getName());
                openInterdictionOfPointRemovalMessage(point->getName(), robot->getRobot()->getName());
                qDebug() << "Sorry this point is the home of a robot and therefore cannot be removed";
            }
        }
        break;
        default:
        /// should never be here
            qDebug() << " dafuk ?";
        break;
    }
}

/**
 * @brief MainWindow::pointInfoEvent
 *
 */
void MainWindow::pointInfoEvent(void){
    qDebug() << "pointinfoevent called";
    /// uncheck the other buttons
    pointsLeftWidget->getPlusButton()->setChecked(false);
    pointsLeftWidget->getMinusButton()->setChecked(false);
    pointsLeftWidget->getEditButton()->setChecked(false);
    pointsLeftWidget->getMapButton()->setChecked(false);

    if(pointsLeftWidget->getEyeButton()->isChecked()){
        pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->setExclusive(true);
        int groupIndex = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedId();
        qDebug() << " my group index guys " << groupIndex;
        /// it's a group
        if(groupIndex != -1 && groupIndex < points.count()-1){
           pointsLeftWidget->getEyeButton()->setChecked(false);
           pointsLeftWidget->hide();
           /// before we display the group of points, we make sure that the graphical object is consistent with the model
           leftMenu->updateGroupDisplayed(points, groupIndex);
           leftMenu->getDisplaySelectedGroup()->show();
           leftMenu->getDisplaySelectedGroup()->setName(points.getGroups().at(groupIndex)->getName());
        }
        /// it's an isolated point
        else if(groupIndex >= points.count()-1){
            qDebug() << "im an isolated point whose info is to be displayed";
            PointView* pointView = pointViews->getPointViewFromPoint(*(points.getGroups().at(points.count()-1)->getPoints().at(groupIndex+1-points.count())));
            DisplaySelectedPoint* selectedPoint = leftMenu->getDisplaySelectedPoint();
            selectedPoint->setPointView(pointView);
            selectedPoint->displayPointInfo();
            if(pointView->getPoint()->isDisplayed())
                selectedPoint->getMapButton()->setChecked(true);
            else
                selectedPoint->getMapButton()->setChecked(false);
            selectedPoint->show();
            pointsLeftWidget->hide();
            leftMenu->getDisplaySelectedGroup()->hide();
        }
    }
}

/**
 * @brief MainWindow::editPointFromGroupMenu
 *
 */
void MainWindow::editPointFromGroupMenu(void){
    qDebug() << "editgroupfrommenuevent";
    std::shared_ptr<Group> group = points.findGroup(leftMenu->getDisplaySelectedGroup()->getNameLabel()->text());
    if(group){
        qDebug() << "working on group " << group->getName() << " and id " << leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->checkedId();
        int point = leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->checkedId();
        if(point != -1 and point < group->getPoints().size()){
            /// update the pointview et show the point on the map with hover color
            leftMenu->getDisplaySelectedPoint()->setPointView(pointViews->getPointViewFromPoint(*(group->getPoints().at(point))));
            leftMenu->getDisplaySelectedPoint()->getPointView()->setPixmap(PointView::PixmapType::HOVER);
            leftMenu->getDisplaySelectedPoint()->getPointView()->show();
            /// update the model
            leftMenu->getDisplaySelectedPoint()->getPointView()->getPoint()->setDisplayed(true);
            /// update the file
            XMLParser parser(XML_PATH, mapPixmapItem);
            parser.save(points);

            leftMenu->getDisplaySelectedPoint()->getNameEdit()->setAutoFillBackground(false);
            leftMenu->getDisplaySelectedPoint()->getNameEdit()->setFrame(true);

            /// if the point is a home point any modification of its name is forbidden
            if(!leftMenu->getDisplaySelectedPoint()->getPoint()->isHome())
                leftMenu->getDisplaySelectedPoint()->getNameEdit()->setReadOnly(false);

            /// sets the state of the map and the other widgets to prevent other concurrent actions
            setGraphicItemsState(GraphicItemState::NO_EVENT, false);
            mapPixmapItem->setState(GraphicItemState::EDITING_PERM);

            /// sets the state of the point of the map to make it draggable
            leftMenu->getDisplaySelectedPoint()->getPointView()->setState(GraphicItemState::EDITING_PERM);
            leftMenu->getDisplaySelectedPoint()->getPointView()->setFlag(QGraphicsItem::ItemIsMovable, true);

            leftMenu->getDisplaySelectedPoint()->getMinusButton()->setEnabled(false);
            leftMenu->getDisplaySelectedPoint()->getMinusButton()->setToolTip("");
            leftMenu->getDisplaySelectedPoint()->getMapButton()->setEnabled(false);
            leftMenu->getDisplaySelectedPoint()->getMapButton()->setToolTip("");
            leftMenu->getDisplaySelectedPoint()->getMapButton()->setChecked(true);
            leftMenu->getDisplaySelectedPoint()->getBackButton()->setEnabled(false);

            /// to force the user to click either the save or the cancel button
            leftMenu->getDisplaySelectedPoint()->getEditButton()->setEnabled(false);
            leftMenu->getDisplaySelectedPoint()->getEditButton()->setToolTip("You can choose to save or discard your modifications by clicking the save (Enter) and cancel button respectively");

            leftMenu->getDisplaySelectedPoint()->setOrigin(DisplaySelectedPoint::GROUP_MENU);

            leftMenu->getDisplaySelectedPoint()->displayPointInfo();

            leftMenu->getDisplaySelectedPoint()->getEditButton()->setChecked(true);
            leftMenu->getDisplaySelectedPoint()->getNameEdit()->setReadOnly(false);
            leftMenu->getDisplaySelectedPoint()->getCancelButton()->show();
            leftMenu->getDisplaySelectedPoint()->getSaveButton()->show();
            leftMenu->getDisplaySelectedPoint()->show();
            leftMenu->getDisplaySelectedGroup()->hide();
        }
    } else qDebug() << "no group " << leftMenu->getDisplaySelectedGroup()->getNameLabel()->text() ;
}

/**
 * @brief MainWindow::displayPointInfoFromGroupMenu
 * display the information of a point from the group menu
 */
void MainWindow::displayPointInfoFromGroupMenu(void){
    qDebug() << "display point info from group menu event called";
    /// retrieves a pointer to the group using the text of the label
    std::shared_ptr<Group> group = points.findGroup(leftMenu->getDisplaySelectedGroup()->getNameLabel()->text());
    if(group){
        /// retrieves the index of the point that is to be displayed, within the group
        int pointIndex = leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->checkedId();
        if(pointIndex != -1 and pointIndex < group->getPoints().size()){
            /// retrieves a pointer to the point itself
            std::shared_ptr<Point> point = group->getPoints().at(pointIndex);
            qDebug() << "checked id " << pointIndex;
            DisplaySelectedPoint* selectedPoint = leftMenu->getDisplaySelectedPoint();
            /// we set the origin of the page displayed, this is useful to implement a "return to the last page" button
            selectedPoint->setOrigin(DisplaySelectedPoint::GROUP_MENU);
            selectedPoint->setPointView(pointViews->getPointViewFromPoint(*point));
            selectedPoint->displayPointInfo();
            /// map is checked if the point is displayed
            if(point->isDisplayed())
                selectedPoint->getMapButton()->setChecked(true);
            else
                selectedPoint->getMapButton()->setChecked(false);
            selectedPoint->show();
            leftMenu->getDisplaySelectedGroup()->hide();
        }
    } else qDebug() << "no group " << leftMenu->getDisplaySelectedGroup()->getNameLabel()->text() ;
}

/**
 * @brief MainWindow::updatePoint
 * called when a user edits a point and save the changes either by pressing the enter key or clicking the save button
 */
void MainWindow::updatePoint(void){
    qDebug() << "update point event called";
    ///resets the tooltip of the edit button and the minus button
    leftMenu->getDisplaySelectedPoint()->getEditButton()->setToolTip("You can click on this button and then choose between clicking on the map or drag the point to change its position");
    leftMenu->getDisplaySelectedPoint()->getMinusButton()->setToolTip("You can click this button to remove the point");

    leftMenu->getDisplaySelectedPoint()->getMapButton()->setEnabled(true);
    leftMenu->getDisplaySelectedPoint()->getMapButton()->setToolTip("Click to hide this point");

    /// resets the color of the pointView
    leftMenu->getDisplaySelectedPoint()->getPointView()->setPixmap(PointView::PixmapType::NORMAL);

    /// notifies the map that the point's name has changed and that the hover has to be updated
    emit nameChanged(leftMenu->getDisplaySelectedPoint()->getPoint()->getName(), leftMenu->getDisplaySelectedPoint()->getNameEdit()->text());

    /// updates the name in the label
    DisplaySelectedPoint* selectedPoint = leftMenu->getDisplaySelectedPoint();
    selectedPoint->getPoint()->setName(leftMenu->getDisplaySelectedPoint()->getNameEdit()->text());

    /// updates the position of the point
    selectedPoint->getPoint()->setPosition(leftMenu->getDisplaySelectedPoint()->getXLabel()->text().right(4).toFloat(),
                                           leftMenu->getDisplaySelectedPoint()->getYLabel()->text().right(4).toFloat());
    /// save changes to the file
    XMLParser parserPoints(XML_PATH, mapPixmapItem);
    parserPoints.save(points);

    /// to change the aspect of the point name
    leftMenu->getDisplaySelectedPoint()->getNameEdit()->setAutoFillBackground(true);
    leftMenu->getDisplaySelectedPoint()->getNameEdit()->setFrame(false);

    /// so that the name cannot be changed anymore unless you click the edit button again
    selectedPoint->getNameEdit()->setReadOnly(true);

    /// so that you cannot edit a new name unless you click the edit button again
    selectedPoint->getEditButton()->setChecked(false);

    /// we hide the save button and the cancel button
    selectedPoint->getCancelButton()->hide();
    selectedPoint->getSaveButton()->hide();

    /// reset the state of the map so we can click it again
    setGraphicItemsState(GraphicItemState::NO_STATE);

    /// enable the edit button and the minus button again
    leftMenu->getDisplaySelectedPoint()->getEditButton()->setEnabled(true);
    leftMenu->getDisplaySelectedPoint()->getMinusButton()->setEnabled(true);

    /// updates the isolated points in the group menus
    pointsLeftWidget->getGroupButtonGroup()->update(points);

    /// to determine wheter the coordinate is 2 digits long or 3 digits long in order to parse them correctly
    int xLength = leftMenu->getDisplaySelectedPoint()->getXLabel()->text().count();
    int yLength = leftMenu->getDisplaySelectedPoint()->getYLabel()->text().count();

    leftMenu->getDisplaySelectedPoint()->getPointView()->getPoint()->setPosition(
                leftMenu->getDisplaySelectedPoint()->getXLabel()->text().right(xLength-4).toFloat(),
                leftMenu->getDisplaySelectedPoint()->getYLabel()->text().right(yLength-4).toFloat());
    /// we enable the "back" button again
    leftMenu->getDisplaySelectedPoint()->getBackButton()->setEnabled(true);
    leftMenu->getDisplaySelectedPoint()->getBackButton()->setToolTip("");
}

/**
 * @brief MainWindow::cancelEvent
 * called when a user discard the changes made about a point
 */
void MainWindow::cancelEvent(void){
    /// reset the color of the pointView
    leftMenu->getDisplaySelectedPoint()->getPointView()->setPixmap(PointView::PixmapType::NORMAL);

    /// to change the aspect of the point name
    leftMenu->getDisplaySelectedPoint()->getNameEdit()->setAutoFillBackground(true);
    leftMenu->getDisplaySelectedPoint()->getNameEdit()->setFrame(false);

    /// we hide the buttons relative to the edit option and make sure the points properties are not longer modifiable
    leftMenu->getDisplaySelectedPoint()->getNameEdit()->setReadOnly(true);
    leftMenu->getDisplaySelectedPoint()->getEditButton()->setChecked(false);
    leftMenu->getDisplaySelectedPoint()->getCancelButton()->hide();
    leftMenu->getDisplaySelectedPoint()->getSaveButton()->hide();

    /// in case the user had dragged the point around the map or clicked it, this resets the coordinates displayed to the original ones
    leftMenu->getDisplaySelectedPoint()->getXLabel()->setText(QString::number(
                                                                  leftMenu->getDisplaySelectedPoint()->getPointView()->getPoint()->getPosition().getX()));
    leftMenu->getDisplaySelectedPoint()->getYLabel()->setText(QString::number(
                                                                  leftMenu->getDisplaySelectedPoint()->getPointView()->getPoint()->getPosition().getY()));
    /// enable the edit button again and the map button
    leftMenu->getDisplaySelectedPoint()->getEditButton()->setEnabled(true);
    leftMenu->getDisplaySelectedPoint()->getEditButton()->setToolTip("You can click on this button and then choose between clicking on the map or drag the point to change its position");
    leftMenu->getDisplaySelectedPoint()->getMapButton()->setEnabled(true);
    leftMenu->getDisplaySelectedPoint()->getMapButton()->setToolTip("Click to hide this point");

    /// resets the tooltip of the minus button
    leftMenu->getDisplaySelectedPoint()->getMinusButton()->setToolTip("You can click this button to remove the point");
    leftMenu->getDisplaySelectedPoint()->getMinusButton()->setEnabled(true);

    /// reset the state of the map so we can click it again
    setGraphicItemsState(GraphicItemState::NO_STATE);
    leftMenu->getDisplaySelectedPoint()->getPointView()->setPos(static_cast<qreal>(leftMenu->getDisplaySelectedPoint()->getPoint()->getPosition().getX()),
                                                                static_cast<qreal>(leftMenu->getDisplaySelectedPoint()->getPoint()->getPosition().getY()));
    /// reset its name in the hover on the map
    leftMenu->getDisplaySelectedPoint()->getNameEdit()->setText(leftMenu->getDisplaySelectedPoint()->getPoint()->getName());

    /// enable the back button in case we were editing coming from the left menu
    leftMenu->getDisplaySelectedPoint()->getBackButton()->setEnabled(true);
    leftMenu->getDisplaySelectedPoint()->getBackButton()->setToolTip("");


}

/**
 * @brief MainWindow::updateCoordinates
 * @param x
 * @param y
 * updates the coordinates of the selected point
 */
void MainWindow::updateCoordinates(double x, double y){
    leftMenu->getDisplaySelectedPoint()->getXLabel()->setText("X : " + QString::number(x));
    leftMenu->getDisplaySelectedPoint()->getYLabel()->setText("Y : " + QString::number(y));
}

/**
 * @brief MainWindow::removePointFromGroupMenu
 * removes a point which does not belong to the default group, from the group menu
 */
void MainWindow::removePointFromGroupMenu(void){
    int checkedId = leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->checkedId();
    if(checkedId > -1)
        askForDeletePointConfirmation(checkedId);
    else
        qDebug() << "can't remove point with index -1";
    leftMenu->getDisplaySelectedGroup()->getMinusButton()->setChecked(false);
}

/**
 * @brief MainWindow::displayPointFromGroupMenu
 * called when a user displays or hides a point on the map from the group menu
 */
void MainWindow::displayPointFromGroupMenu(){
    qDebug() << "displaypointfromgroupmenu event called";
    int checkedId = leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->checkedId();
    if(checkedId != -1 && checkedId < points.getGroups().at(pointsLeftWidget->getIndexLastGroupClicked())->count()){
        std::shared_ptr<Point> currentPoint = points.getGroups().at(pointsLeftWidget->getIndexLastGroupClicked())->getPoints().at(checkedId);
        /// if the point is displayed we stop displaying it
        if(currentPoint->isDisplayed()){
            /// hides the point on the map
            pointViews->getPointViewFromPoint(*(points.getGroups().at(pointsLeftWidget->getIndexLastGroupClicked())->getPoints().at(checkedId)))->hide();
            /// updates the model
            points.getGroups().at(pointsLeftWidget->getIndexLastGroupClicked())->getPoints().at(checkedId)->setDisplayed(false);
            /// removes the tick icon to show that the point is not displayed on the map
            leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon());
            /// updates the file
            XMLParser parserPoints(XML_PATH, mapPixmapItem);
            parserPoints.save(points);
            /// if the entire group was displayed it is not the case anymore
            pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(pointsLeftWidget->getIndexLastGroupClicked())->setIcon(QIcon(":/icons/folder.png"));
        } else {
            /// shows the point on the map
            pointViews->getPointViewFromPoint(*(points.getGroups().at(pointsLeftWidget->getIndexLastGroupClicked())->getPoints().at(checkedId)))->show();
            /// updates the model
            points.getGroups().at(pointsLeftWidget->getIndexLastGroupClicked())->getPoints().at(checkedId)->setDisplayed(true);
            /// we add a tick icon next to the name of the point to show that it is displayed on the map
            leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/tick.png"));
            /// saves changes to the file
            XMLParser parserPoints(XML_PATH, mapPixmapItem);
            parserPoints.save(points);
            /// we check whether or not the entire group is displayed and update the points left widget accordingly by adding a tick Icon or not
            if(points.getGroups().at(pointsLeftWidget->getIndexLastGroupClicked())->isDisplayed())
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(pointsLeftWidget->getIndexLastGroupClicked())->setIcon(QIcon(":/icons/folder_tick.png"));
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
 * @brief MainWindow::doubleClickOnPoint
 * @param checkedId
 * does the same as clicking on a point and then on the eye button
 */
void MainWindow::doubleClickOnPoint(int checkedId){
    std::shared_ptr<Group> group = points.findGroup(leftMenu->getDisplaySelectedGroup()->getNameLabel()->text());
    if(group){
        std::shared_ptr<Point> point = group->getPoints().at(checkedId);
        if(checkedId != -1 and checkedId < group->getPoints().size()){
            DisplaySelectedPoint* selectedPoint = leftMenu->getDisplaySelectedPoint();
            selectedPoint->setOrigin(DisplaySelectedPoint::GROUP_MENU);
            selectedPoint->setPointView(pointViews->getPointViewFromPoint(*point));
            selectedPoint->displayPointInfo();
            if(point->isDisplayed())
                selectedPoint->getMapButton()->setChecked(true);
            else
                selectedPoint->getMapButton()->setChecked(false);
            selectedPoint->show();
            leftMenu->getDisplaySelectedGroup()->hide();
        }
    } else qDebug() << "no group " << leftMenu->getDisplaySelectedGroup()->getNameLabel()->text() ;
}

/**
 * @brief MainWindow::doubleClickOnGroup
 * @param checkedId
 * does the same as clicking on a group or a point belonging to the default group
 * and then on the eye button
 */
void MainWindow::doubleClickOnGroup(int checkedId){
    qDebug() << " double click on group or defaul point ";
    /// uncheck the other buttons
    pointsLeftWidget->getPlusButton()->setChecked(false);
    pointsLeftWidget->getMinusButton()->setChecked(false);
    pointsLeftWidget->getEditButton()->setChecked(false);
    pointsLeftWidget->getMapButton()->setChecked(false);

    /// we hide those in case the previous button clicked was the plus button
    pointsLeftWidget->getGroupNameEdit()->hide();
    pointsLeftWidget->getGroupNameLabel()->hide();

    /// it's a group
    if(checkedId != -1 && checkedId < points.count()-1){
       pointsLeftWidget->setIndexLastGroupClicked(checkedId);
       pointsLeftWidget->getEyeButton()->setChecked(false);
       pointsLeftWidget->hide();
       /// before we display the group of points, we make sure that the graphical object is consistent with the model
       DisplaySelectedGroup* selectedGroup = leftMenu->getDisplaySelectedGroup();
       leftMenu->updateGroupDisplayed(points, checkedId);
       selectedGroup->getPointButtonGroup()->setCheckable(true);
       selectedGroup->show();
       selectedGroup->setName(points.getGroups().at(checkedId)->getName());
    }
    /// it's an isolated point
    else if(checkedId >= points.count()-1){
        DisplaySelectedPoint* selectedPoint = leftMenu->getDisplaySelectedPoint();
        PointView* pointView = pointViews->getPointViewFromPoint(*(points.getGroups().at(points.count()-1)->getPoints().at(checkedId+1-points.count())));
        selectedPoint->setPointView(pointView);
        selectedPoint->displayPointInfo();
        selectedPoint->show();
        pointsLeftWidget->getEyeButton()->setChecked(false);
        if(pointView->getPoint()->isDisplayed())
            selectedPoint->getMapButton()->setChecked(true);
        else
            selectedPoint->getMapButton()->setChecked(false);
        pointsLeftWidget->hide();
    }
}

/**
 * @brief MainWindow::reestablishConnections
 * to reestablish the double clicks after points are updated (because buttons in the menu are recreated)
 */
void MainWindow::reestablishConnectionsGroups(){
    qDebug() << " connections for groups requested";
    foreach(QAbstractButton* button, pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->buttons())
        connect(button, SIGNAL(doubleClick(int)), this, SLOT(doubleClickOnGroup(int)));
}

void MainWindow::reestablishConnectionsPoints(){
    qDebug() << "connections for points requested";
    foreach(QAbstractButton* button, leftMenu->getDisplaySelectedGroup()->getPointButtonGroup()->getButtonGroup()->buttons())
        connect(button, SIGNAL(doubleClick(int)), this, SLOT(doubleClickOnPoint(int)));
}

/**********************************************************************************************************************************/

//                                          ODDS AND ENDS

/**********************************************************************************************************************************/

void MainWindow::quit(){
    close();
}

QList<QWidget*> MainWindow::getLastWidget(void)
{
    return lastWidget;
}

void MainWindow::setLastWidget(QList<QWidget*> lw)
{
     lastWidget = lw;
}
QWidget* MainWindow::getCurrentWidget(void)
{
    return currentWidget;
}
void MainWindow::setCurrentWidget(QWidget* cw)
{
     currentWidget = cw;
}



QList<QString> MainWindow::getLastName(void)
{
    return lastName;
}
void MainWindow::setLastName(QList<QString> ln)
{
     lastName = ln;
}
QString MainWindow::getCurrentName(void)
{
    return currentName;
}
void MainWindow::setCurrentName(QString cn)
{
     currentName = cn;
}

void MainWindow::backEvent()
{
    qDebug() << "back event called";

    currentWidget->hide();
    if (lastWidget.size() != 0)
    {
        currentWidget = lastWidget.takeLast();
        currentWidget->show();
        currentName = lastName.takeLast();
    }
    else
    {
        currentWidget = NULL;
        leftMenu->hide();
    }
    if (lastWidget.size() != 0)
    {
        leftMenu->showBackButton(&lastName.last());
    }
    else
    {
        leftMenu->hideBackButton();
    }

}




/**
 * @brief MainWindow::openConfirmMessage
 * @param text
 * @return int
 * prompts the user for confirmation
 */
int MainWindow::openConfirmMessage(const QString text){
    QMessageBox msgBox;
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
    mapPixmapItem->setState(state, clear);

    for(int i = 0; i < robots->getRobotsVector().size(); i++){
        robots->getRobotsVector().at(i)->setState(state);
    }

    for(size_t j = 0; j < pointViews->count(); j++){
        GroupView* groupView = pointViews->getGroups().at(j);
        for(size_t k = 0; k < groupView->getPointViews().size(); k++){
            groupView->getPointViews().at(k)->setState(state);
        }
    }
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

void MainWindow::clearNewMap(){
    qDebug() << "clearNewMap called";

    /// Clear all the paths
    for(int i = 0; i < robots->getRobotsVector().size(); i++){
        clearPath(i);
        robots->getRobotsVector().at(i)->getRobot()->setHome(NULL);
    }

    selectedPoint = NULL;
    editedPointView = NULL;

    /// Clear the list of points
    points.clear();

    /// Save the new list in the XML
    XMLParser parserPoints(XML_PATH, mapPixmapItem);
    parserPoints.save(points);

    /// Delete the pointView list
    delete pointViews;
    pointViews = new PointsView(points, mapPixmapItem);

    /// Update the left menu displaying the list of groups and buttons
    pointsLeftWidget->updateGroupButtonGroup(points);

    /// Update the map
    mapPixmapItem->setPermanentPoints(points);

}
