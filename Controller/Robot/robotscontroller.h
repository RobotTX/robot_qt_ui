#ifndef ROBOTSCONTROLLER_H
#define ROBOTSCONTROLLER_H

class RobotController;
class Robots;
class MainController;
class RobotServerWorker;
class QQmlApplicationEngine;
class Points;

#include <QObject>
#include <QPointer>
#include <QMap>
#include <QThread>
#include <QVariant>
#include <QTimer>
#include <QImage>
#include "Controller/Robot/backupcontroller.h"


class RobotsController : public QObject {

    Q_OBJECT

public:

    RobotsController(QObject *applicationWindow, QQmlApplicationEngine *engine, MainController *parent);
    ~RobotsController();

    QMap<QString, QPointer<RobotController>> getRobots(void) const { return robots; }
    QString getLangue(void) const { return langue; }

    /**
     * @brief setRobotPos
     * @param ip
     * @param posX
     * @param posY
     * @param ori
     * Send the robot pos to the qml model
     */
    void setRobotPos(const QString ip, const double posX, const double posY, const double ori);

    /**
     * @brief sendCommand
     * @param ip
     * @param cmd
     * @return true if we could send the command
     * Send the command to the robotController
     */
    bool sendCommand(const QString ip, const QString cmd);

    bool sendMP3(const QString ip, const QStringList mp3Str);

    /**
     * @brief sendNewMap
     * @param ip
     * @param mapId
     * @param date
     * @param mapMetadata
     * @param mapImage
     * sends a new map to a robot and resets its path and home
     */
    void sendNewMap(const QString ip, const QString mapId, const QString date, const QString mapMetadata, const QImage mapImage);
    /**
     * @brief requestMap
     * @param ip
     * requests map from robot at ip <ip>
     */
    void requestMap(const QString ip);
    /**
     * @brief sendNewMapToAllExcept
     * @param ip
     * @param mapId
     * @param date
     * @param mapMetadata
     * @param mapImage
     * sends a new map to all robots except the one at ip <ip>
     */
    void sendNewMapToAllExcept(const QString ip, const QString mapId, const QString date, const QString mapMetadata, const QImage mapImage);
    /**
     * @brief requestMapForMerging
     * @param ip
     * requests a map for merging from robot at ip <ip>
     */
    void requestMapForMerging(const QString ip);
    /**
     * @brief sendTeleop
     * @param ip
     * @param teleop
     * sends a teleoperation command to robot at ip <ip>
     */
    void sendTeleop(const QString ip, const int teleop);
    /**
     * @brief sendMapToAllRobots
     * @param mapId
     * @param date
     * @param mapMetadata
     * @param img
     * sends the map to all robots
     */
    void sendMapToAllRobots(QString mapId, QString date, QString mapMetadata, QImage img);

    void sendMP3ToRobot(QString fileName, bool isLastMP3File);

//    void changeLanguageSlot(const QString language);
private:
    void launchServer(void);

private slots:
    /**
     * @brief robotIsAliveSlot
     * @param name
     * @param ip
     * @param stage
     * @param battery
     * Received several times per second to notify that the robot at address <ip>
     * is still connected with the robot, carries useful information such as path stage and battery level
     * along the way
     */
    void robotIsAliveSlot(const QString name, const QString ip, const int stage, const int battery, const bool charging, const int dockStatus, const int robotMode);
    /**
     * @brief robotIsDeadSlot
     * @param ip
     */
    void robotIsDeadSlot(const QString ip);
    void newRobotPosSlot(const QString ip, const double posX, const double posY, const double ori);
    void sendCommandNewName(const QString ip, const QString name);
    void updateNameSlot(const QString ip, const QString name);
    void sendCommandDeletePath(const QString ip);
    void stoppedDeletedPathSlot(const QString ip);
    void sendCommandPausePath(const QString ip);
    void sendCommandPlayPath(const QString ip);
    void sendCommandStopPath(const QString ip);
    void updatePlayingPathSlot(const QString ip, const bool playingPath);
    void newMapFromRobotSlot(const QString ip, const QByteArray mapArray, const QString mapId, const QString mapDate, const QString resolution, const QString originX, const QString originY, const int map_width, const int map_height);
    void sendMapTimerSlot(void);
    void requestMapTimerSlot(void);

    void stoppedScanningSlot(const QString ip);
    void pausedScanningSlot(const QString ip);
    void playedExplorationSlot(const QString ip);
    void pausedExplorationSlot(const QString ip);
    void receivedScanMapSlot(const QString ip, const QByteArray map, const QString resolution, const QString originX, const QString originY, const int map_width, const int map_height);
    void processingCmdSlot(QString ip, bool processing);
    void setMessageTopSlot(int status, QString msg);
    void activateLaserSlot(QString ip, bool activate);
    void startedScanningSlot(const QString ip);

    void updateRobotPos(QString ip, double x, double y, double orientation);

    void startDockingRobot(QString ip);
    void stopDockingRobot(QString ip);
    void resetHomePathSlot(QString ip);

    /**
     * @brief shortcutAddRobot
     * to add and remove mock robots to the application
     */
    void shortcutAddRobot(void);
    void shortcutDeleteRobot(void);
    void callForRebootRobot(QString ip);
    void backupSystemIsDownSlot(QString ip);
    void setCmdLoopingSlot(QString ip, bool loop);

    void soundOn(QString ip);
    void soundOff(QString ip);

    void decreaseSound(QString ip);
    void increaseSound(QString ip);

    /**
     * @brief updateRobotInfoSlot
     * @param ip
     * @param robotInfo
     * Update the info when a new robot connects
     */
    void updateRobotInfoSlot(QString ip, QString robotInfo);

    void updatePathSlot(const QString ip, const QStringList strList);

    void startAudioTransfertSlot();

    void updateHomeSlot(const QString ip, const double homeX, const double homeY, const double homeOri);

     void updateLinearVelocitySlot(QString ip, double linear);

    void updateLaserSlot(QString ip, bool activated);

    void setLoopingSlot(QString ip, bool looping);

    void setVelocitySlot(QString ip, double linear, double angular);

    void setBatteryWarningSlot(QString ip, double batteryLevel);

    void interruptDelay(QString ip);

signals:
    void stopRobotServerWorker(void);
    void addRobot(QVariant name, QVariant ip, QVariant stage, QVariant battery);
    void removeRobot(QVariant ip);
    void setPos(QVariant ip, QVariant posX, QVariant posY, QVariant orientation);
//    void setTts(QVariant ip, QVariant tts);
    void setHome(QVariant ip, QVariant posX, QVariant posY, QVariant homeOri);
    void setLinearVelocity(QVariant ip, QVariant linear);
    void setPath(QVariant ip, QVariant name);
    void setPlayingPath(QVariant ip, QVariant playingPath);
    void addPathPoint(QVariant ip, QVariant name, QVariant posX, QVariant posY, QVariant waitTime, QVariant orientation, QVariant speechName, QVariant speechContent, QVariant speechTime);
    void displayRobots(void);
    void setStage(QVariant ip, QVariant stage);
    void setBattery(QVariant ip, QVariant battery, QVariant charging);
    void newRobotPos(QString ip, double posX, double posY, double ori);
    void updatePath(QString ip, QStringList strList);
    void updateHome(QString ip, double homeX, double homeY, double homeOri);
    void updateLinearVelocity(QString ip, double linear);
    void setName(QVariant ip, QVariant name);
    void checkMapInfo(QString ip, QString mapId, QString mapDate);
    void newMapFromRobot(QString ip, QByteArray mapArray, QString mapId, QString mapDate, QString resolution, QString originX, QString originY, int map_width, int map_height);
    void stoppedScanning(QVariant ip);
    void startedScanning(QVariant ip);
    void pausedScanning(QVariant ip);
    void receivedScanMap(QString ip, QByteArray mapArray, QString resolution, QString originX, QString originY, int map_width, int map_height);
    void setScanningOnConnection(QVariant ip, QVariant scanningOnConnection);
    void checkScanWindow(QVariant ip, QVariant scanning);
    void removeScanMap(QString);
    void processingCmd(QVariant, QVariant);
    void testScanSignal(QString);
    void setMessageTop(int status, QString msg);
    void updateLaser(QVariant ip, QVariant activated);
    void updateDockStatus(QVariant ip, QVariant status);
    void resetHome(QVariant ip);
    void playedExploration(QVariant ip);
    void pausedExploration(QVariant ip);
    void setLooping(QVariant ip, QVariant looping);
    void setVelocity(QVariant ip, QVariant linear, QVariant angular);
    void playingPathSlot(QVariant ip, QVariant playing);
    void updateSound(QVariant ip, QVariant activate);
    void updateRobotMode(QVariant ip, QVariant robotMode);
    void setBatteryWarning(QVariant ip, QVariant batteryLevelWarning);

private:
    QQmlApplicationEngine* engine_;
    QMap<QString, QPointer<RobotController>> robots;
    //QMap<QString, QPointer<BackupController>> backupControllers;
    QPointer<RobotServerWorker> robotServerWorker;
    QThread serverThread;
    bool receivingMap;
    QPointer<QTimer> sendMapTimer;
    QPointer<QTimer> requestMapTimer;
    QString langue;
    QString _ipRobot;
    QStringList _mp3Str;
};

#endif /// ROBOTSCONTROLLER_H
