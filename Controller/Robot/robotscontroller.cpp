#include "robotscontroller.h"
#include "Helper/helper.h"
#include "Controller/Robot/robotcontroller.h"
#include "Controller/Robot/robotserverworker.h"
#include "Controller/maincontroller.h"

RobotsController::RobotsController(QObject *applicationWindow, MainController* parent) : QObject(parent), robots(QMap<QString, QPointer<RobotController>>()){

    QObject *robotModel = applicationWindow->findChild<QObject*>("robotModel");
    if (robotModel){
        connect(this, SIGNAL(displayRobots()), robotModel, SLOT(display()));
        connect(this, SIGNAL(addRobot(QVariant, QVariant, QVariant, QVariant, QVariant)),
                robotModel, SLOT(addRobot(QVariant, QVariant, QVariant, QVariant, QVariant)));
        connect(this, SIGNAL(removeRobot(QVariant)),
                robotModel, SLOT(removeRobot(QVariant)));
        connect(this, SIGNAL(setPos(QVariant, QVariant, QVariant, QVariant)),
                robotModel, SLOT(setPos(QVariant, QVariant, QVariant, QVariant)));
        connect(this, SIGNAL(setHome(QVariant, QVariant, QVariant, QVariant)),
                robotModel, SLOT(setHome(QVariant, QVariant, QVariant, QVariant)));
        connect(this, SIGNAL(setPath(QVariant, QVariant)), robotModel, SLOT(setPath(QVariant, QVariant)));
        connect(this, SIGNAL(setPlayingPath(QVariant, QVariant)), robotModel, SLOT(setPlayingPath(QVariant, QVariant)));
        connect(this, SIGNAL(addPathPoint(QVariant, QVariant, QVariant, QVariant, QVariant)),
                robotModel, SLOT(addPathPoint(QVariant, QVariant, QVariant, QVariant, QVariant)));
        connect(this, SIGNAL(setStage(QVariant, QVariant)), robotModel, SLOT(setStage(QVariant, QVariant)));
        connect(this, SIGNAL(setBattery(QVariant, QVariant)), robotModel, SLOT(setBattery(QVariant, QVariant)));
    } else {
        qDebug() << "RobotsController::RobotsController could not find the qml robot model";
        Q_UNREACHABLE();
    }

    launchServer();
}

RobotsController::~RobotsController(){
    if(robotServerWorker){
        emit stopRobotServerWorker();
        serverThread.quit();
        serverThread.wait();
    }
/*
    QMapIterator<QString, QPointer<RobotController>> i(robots);
    while (i.hasNext()) {
        i.next();
        i.value()->stopThreads();
    }*/
}

void RobotsController::launchServer(){
    robotServerWorker = new RobotServerWorker(PORT_ROBOT_UPDATE);
    connect(robotServerWorker, SIGNAL(robotIsAlive(QString, QString, QString, int, int)), this, SLOT(robotIsAliveSlot(QString, QString, QString, int, int)));
    connect(this, SIGNAL(stopRobotServerWorker()), robotServerWorker, SLOT(stopWorker()));
    connect(&serverThread, SIGNAL(finished()), robotServerWorker, SLOT(deleteLater()));
    serverThread.start();
    robotServerWorker->moveToThread(&serverThread);
}

void RobotsController::robotIsAliveSlot(QString name, QString ip, QString ssid, int stage, int battery){
    qDebug() << "RobotsController::robotIsAliveSlot" << name << ip << ssid << stage << battery;
    if(robots.find(ip) != robots.end()){
        /// TODO update battery + stage + ping RobotController
    } else {
        QPointer<RobotController> robotController = QPointer<RobotController>(new RobotController(this, ip));
        robots.insert(ip, robotController);
        connect(robotController, SIGNAL(robotIsDead(QString)), this, SLOT(robotIsDeadSlot(QString)));
        emit addRobot(QVariant::fromValue(name), QVariant::fromValue(ip), QVariant::fromValue(ssid), QVariant::fromValue(stage), QVariant::fromValue(battery));
    }
}

void RobotsController::robotIsDeadSlot(QString ip){
    robots.remove(ip);
    emit removeRobot(ip);
}

void RobotsController::shortcutAddRobot(){
    QString ip = QString::number(robots.size());
    double posX = ((robots.size() + 1) * 200) % 1555;
    double posY = ((robots.size() + 1) * 200) % 1222;

    robotIsAliveSlot("Robot " + ip, ip, "Wifi " + ip, 0, (robots.size()*10)%100);
    emit setPos(QVariant::fromValue(ip), QVariant::fromValue(posX), QVariant::fromValue(posY), QVariant::fromValue((20 * robots.size()) % 360));

    if((robots.size() - 1)%2 == 0){
        emit setHome(QVariant::fromValue(ip), QVariant::fromValue("home " + ip), QVariant::fromValue(posX + 50), QVariant::fromValue(posY));
        emit setPlayingPath(QVariant::fromValue(ip), QVariant::fromValue((robots.size() - 1)%4/2));
    }

    if((robots.size() - 1)%3 == 0){
        emit setPath(QVariant::fromValue(ip), QVariant::fromValue("pathName " + ip));
        emit addPathPoint(QVariant::fromValue(ip), QVariant::fromValue(QString("pathPoint 1")), QVariant::fromValue(posX + 50 * robots.size()), QVariant::fromValue(posY + 50 * robots.size()), QVariant::fromValue((robots.size() - 1)%3));
        emit addPathPoint(QVariant::fromValue(ip), QVariant::fromValue(QString("pathPoint 2")), QVariant::fromValue(posX + 50 * robots.size()*2), QVariant::fromValue(posY + 50 * robots.size()*2), QVariant::fromValue((robots.size() - 1)%3));
        emit addPathPoint(QVariant::fromValue(ip), QVariant::fromValue(QString("pathPoint 3")), QVariant::fromValue(posX + 50 * robots.size()*3), QVariant::fromValue(posY + 50 * robots.size()*3), QVariant::fromValue((robots.size() - 1)%3));
        emit setStage(QVariant::fromValue(ip), QVariant::fromValue((int) ((robots.size() - 1) / 3)));
    }
    emit displayRobots();
}

void RobotsController::shortcutDeleteRobot(){
    if(robots.size() > 0){
        robotIsDeadSlot(QString::number(robots.size() - 1));
        emit displayRobots();
    } else
        qDebug() << "You already have no robot";
}
