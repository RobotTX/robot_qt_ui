#include "robotscontroller.h"
#include <QDir>
#include <assert.h>
#include "Helper/helper.h"
#include "Controller/Robots/robotserverworker.h"
#include "Controller/Points/pointscontroller.h"
#include "Controller/Map/mapcontroller.h"
#include "Controller/Paths/pathscontroller.h"
#include "Model/Paths/paths.h"
#include "Model/Robots/robots.h"
#include "View/Robots/robotview.h"
#include "View/Robots/robotsleftwidget.h"
#include "View/Robots/editselectedrobotwidget.h"

RobotsController::RobotsController(MainWindow *mainWindow) : QObject(mainWindow){
    robots = QSharedPointer<Robots>(new Robots());
    selectedRobot = Q_NULLPTR;

    /// Get the list of taken robot's name from the file
    QFile fileRead(QDir::currentPath() + QDir::separator() + "robotsName.dat");

    fileRead.open(QIODevice::ReadWrite);
    /// read the data serialized from the file
    QDataStream in(&fileRead);
    QMap<QString, QString> tmp;
    in >> tmp;
    robots->setRobotsNameMap(tmp);
    fileRead.close();
}

RobotsController::~RobotsController(){
    if(robotServerWorker){
        emit stopUpdateRobotsThread();
        serverThread.quit();
        serverThread.wait();
    }
}

void RobotsController::launchServer(MainWindow* mainWindow){
    robotServerWorker = new RobotServerWorker(PORT_ROBOT_UPDATE);
    connect(robotServerWorker, SIGNAL(robotIsAlive(QString, QString, QString, int, int)), mainWindow, SLOT(robotIsAliveSlot(QString, QString, QString, int, int)));
    connect(this, SIGNAL(stopUpdateRobotsThread()), robotServerWorker, SLOT(stopWorker()));
    connect(&serverThread, SIGNAL(finished()), robotServerWorker, SLOT(deleteLater()));
    serverThread.start();
    robotServerWorker->moveToThread(&serverThread);
}

void RobotsController::initializeMenus(MainWindow* mainWindow){

    /// Menu which display the list of robots
    robotsLeftWidget = new RobotsLeftWidget(mainWindow);
    robotsLeftWidget->hide();
    connect(robotsLeftWidget, SIGNAL(deselectRobots()), this, SLOT(delesectRobotsSlot()));

    /// Menu to edit the selected robot
    editSelectedRobotWidget = new EditSelectedRobotWidget(mainWindow);
    editSelectedRobotWidget->hide();
    connect(editSelectedRobotWidget, SIGNAL(showEditSelectedRobotWidget()), mainWindow, SLOT(showHome()));
    connect(editSelectedRobotWidget, SIGNAL(hideEditSelectedRobotWidget()), mainWindow, SLOT(showAllHomes()));
    connect(editSelectedRobotWidget, SIGNAL(updatePathsMenu(bool)), this, SLOT(updatePathsMenuEditSelectedRobotWidget(bool)));
    connect(editSelectedRobotWidget, SIGNAL(updateHomeMenu(bool)), this, SLOT(updateHomeMenuEditSelectedRobotWidget(bool)));

    connect(mainWindow, SIGNAL(updatePath(QString, QString)), this, SLOT(applyNewPath(QString, QString)));
    /// to display a path that's assigned to the robot after clearing the map of other path(s)
    connect(this, SIGNAL(clearMapOfPaths()), mainWindow, SLOT(clearMapOfPaths()));
    connect(this, SIGNAL(showPath(QString, QString)), mainWindow, SLOT(displayAssignedPath(QString, QString)));
}

void RobotsController::updateRobot(const QString ipAddress, const float posX, const float posY, const float oriZ){

    MainWindow* mainWindow = static_cast<MainWindow*>(parent());
    /// need to first convert the coordinates that we receive from the robot
    Position robotPositionInPixelCoordinates = Helper::Convert::robotCoordToPixelCoord(
                Position(posX, posY),
                mainWindow->getMapController()->getMap()->getOrigin().getX(),
                mainWindow->getMapController()->getMap()->getOrigin().getY(),
                mainWindow->getMapController()->getMap()->getResolution(),
                mainWindow->getMapController()->getMap()->getHeight());
    float orientation = -oriZ * 180.0 / PI + 90;

    QPointer<RobotView> robotView = robots->getRobotViewByIp(ipAddress);
    if(robotView){
        robotView->setPosition(robotPositionInPixelCoordinates.getX(), robotPositionInPixelCoordinates.getY());
        robotView->setOrientation(orientation);

        emit scanRobotPos(robotView->getRobot()->getName(), robotPositionInPixelCoordinates.getX(), robotPositionInPixelCoordinates.getY(), orientation);
    }
}

void RobotsController::updatePathsMenuEditSelectedRobotWidget(bool openMenu){
    editSelectedRobotWidget->updatePathsMenu(static_cast<MainWindow*>(parent()));
    if(openMenu)
        editSelectedRobotWidget->openPathsMenu();
}

void RobotsController::updateHomeMenuEditSelectedRobotWidget(bool openMenu){
    editSelectedRobotWidget->updateHomeMenu(static_cast<MainWindow*>(parent())->getPointsController()->getPoints()->getGroups());
    if(openMenu)
        editSelectedRobotWidget->openHomeMenu();
}

void RobotsController::applyNewPath(const QString groupName, const QString pathName){
    qDebug() << "EditSelectedRobotWidget::applyNewPath " << pathName;
    /// Once we know for sure (from the main window) that the command has been received, we proceed to the update here
    emit clearMapOfPaths();
    bool foundFlag(false);
    editSelectedRobotWidget->setPath(static_cast<MainWindow*>(parent())->getPathsController()->getPaths()->getPath(groupName, pathName, foundFlag));
    assert(foundFlag);
    emit showPath(groupName, pathName);
}

void RobotsController::updateRobotsLeftWidget(void){
    robotsLeftWidget->getBtnGroup()->updateRobots(robots);
}

void RobotsController::delesectRobotsSlot(){
    robots->deselect();
}
