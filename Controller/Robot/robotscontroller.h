#ifndef ROBOTSCONTROLLER_H
#define ROBOTSCONTROLLER_H

class RobotController;
class MainController;
class RobotServerWorker;

#include <QObject>
#include <QPointer>
#include <QMap>
#include <QThread>
#include <QVariant>
#include <QTimer>

class RobotsController : public QObject {

    Q_OBJECT

public:

    RobotsController(QObject *applicationWindow, MainController *parent);
    ~RobotsController();

    void setRobotPos(QString ip, float posX, float posY, float ori);
    void sendCommand(QString ip, QString cmd);
    void sendNewMap(QString ip, QString mapId, QString date, QString mapMetadata, QImage mapImage);
    void requestMap(QString ip);
    void sendNewMapToAllExcept(QString ip, QString mapId, QString date, QString mapMetadata, QImage mapImage);
    void requestMapForMerging(QString ip);

private:
    void launchServer();

private slots:

    void robotIsAliveSlot(QString name, QString ip, QString ssid, int stage, int battery);
    void robotIsDeadSlot(QString ip);
    void shortcutAddRobot();
    void shortcutDeleteRobot();
    void newRobotPosSlot(QString ip, float posX, float posY, float ori);
    void newMetadataSlot(int width, int height, float resolution, float originX, float originY);
    void updatePathSlot(QString ip, QStringList strList);
    void updateHomeSlot(QString ip, QString homeName, float homeX, float homeY);
    void sendCommandNewName(QString ip, QString name);
    void updateNameSlot(QString ip, QString name);
    void sendCommandDeletePath(QString ip);
    void stoppedDeletedPathSlot(QString ip);
    void sendCommandPausePath(QString ip);
    void sendCommandPlayPath(QString ip);
    void sendCommandStopPath(QString ip);
    void updatePlayingPathSlot(QString ip, bool playingPath);
    void checkMapInfoSlot(QString ip, QString mapId, QString mapDate);
    void newMapFromRobotSlot(QString ip, QByteArray mapArray, QString mapId, QString mapDate);
    void timerSlot();
    void processMapForMerge(QByteArray map, QString resolution);
    void startedScanningSlot(QString ip);
    void stoppedScanningSlot(QString ip);
    void pausedScanningSlot(QString ip);
    void receivedScanMapSlot(QString ip, QByteArray map, QString resolution);

signals:
    void stopRobotServerWorker();
    void addRobot(QVariant name, QVariant ip, QVariant ssid, QVariant stage, QVariant battery);
    void removeRobot(QVariant ip);
    void setPos(QVariant ip, QVariant posX, QVariant posY, QVariant orientation);
    void setHome(QVariant ip, QVariant name, QVariant posX, QVariant posY);
    void setPath(QVariant ip, QVariant name);
    void setPlayingPath(QVariant, QVariant);
    void addPathPoint(QVariant ip, QVariant name, QVariant posX, QVariant posY, QVariant waitTime);
    void displayRobots();
    void setStage(QVariant, QVariant);
    void setBattery(QVariant, QVariant);
    void newRobotPos(QString ip, float posX, float posY, float ori);
    void newMetadata(int width, int height, float resolution, float originX, float originY);
    void updatePath(QString ip, QStringList strList);
    void updateHome(QString ip, QString homeName, float homeX, float homeY);
    void setName(QVariant ip, QVariant name);
    void checkMapInfo(QString ip, QString mapId, QString mapDate);
    void newMapFromRobot(QString ip, QByteArray mapArray, QString mapId, QString mapDate);
    void sendMapToProcessForMerge(QByteArray, QString);
    void stoppedScanning(QVariant);
    void startedScanning(QVariant);
    void pausedScanning(QVariant);
    void receivedScanMap(QString ip, QByteArray mapArray, QString resolution);

private:
    QMap<QString, QPointer<RobotController>> robots;

    QPointer<RobotServerWorker> robotServerWorker;
    QThread serverThread;
    bool receivingMap;
    QPointer<QTimer> timer;
};

#endif /// ROBOTSCONTROLLER_H
