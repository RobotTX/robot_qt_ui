#include "robotscontroller.h"
#include "Controller/robotserverworker.h"
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

    /// Menu which display the list of robots
    robotsLeftWidget = new RobotsLeftWidget(mainWindow, robots);
    robotsLeftWidget->hide();

    /// Menu to edit the selected robot
    editSelectedRobotWidget = new EditSelectedRobotWidget(this, mainWindow, points, robots, mainWindow->getPathsController()->getPaths());
    editSelectedRobotWidget->hide();
    connect(editSelectedRobotWidget, SIGNAL(showEditSelectedRobotWidget()), mainWindow, SLOT(showEditHome()));
    connect(editSelectedRobotWidget, SIGNAL(hideEditSelectedRobotWidget()), mainWindow, SLOT(showAllHomes()));
}

RobotsController::~RobotsController(){
    if(robotServerWorker){
        emit stopUpdateRobotsThread();
        serverThread.quit();
        serverThread.wait();
    }
}
