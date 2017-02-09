#include "robotscontroller.h"
#include <QDir>
#include "Helper/helper.h"
#include "Controller/robotserverworker.h"
#include "Controller/pointscontroller.h"
#include "Controller/mapcontroller.h"
#include "Model/robots.h"
#include "View/robotview.h"
#include "View/robotsleftwidget.h"
#include "View/editselectedrobotwidget.h"

RobotsController::RobotsController(MainWindow *mainWindow) : QObject(mainWindow){
    robots = QSharedPointer<Robots>(new Robots());
    selectedRobot = Q_NULLPTR;

    robotServerWorker = new RobotServerWorker(PORT_ROBOT_UPDATE);
    connect(robotServerWorker, SIGNAL(robotIsAlive(QString, QString, QString, int, int)), mainWindow, SLOT(robotIsAliveSlot(QString, QString, QString, int, int)));
    connect(this, SIGNAL(stopUpdateRobotsThread()), robotServerWorker, SLOT(stopWorker()));
    connect(&serverThread, SIGNAL(finished()), robotServerWorker, SLOT(deleteLater()));
    serverThread.start();
    robotServerWorker->moveToThread(&serverThread);


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

void RobotsController::initializeMenus(MainWindow* mainWindow){

    /// Menu which display the list of robots
    robotsLeftWidget = new RobotsLeftWidget(mainWindow, robots);
    robotsLeftWidget->hide();

    /// Menu to edit the selected robot
    editSelectedRobotWidget = new EditSelectedRobotWidget(mainWindow);
    editSelectedRobotWidget->hide();
    connect(editSelectedRobotWidget, SIGNAL(showEditSelectedRobotWidget()), mainWindow, SLOT(showEditHome()));
    connect(editSelectedRobotWidget, SIGNAL(hideEditSelectedRobotWidget()), mainWindow, SLOT(showAllHomes()));
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
