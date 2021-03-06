#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

class CmdRobotWorker;
class RobotPositionWorker;
class SendNewMapWorker;
//class LaserWorker;
class ScanMapWorker;
class TeleopWorker;
class CommandController;
class RobotsController;
class ObstaclesPaintedItem;
class QQmlApplicationEngine;

#include <QObject>
#include <QThread>
#include <QPointer>
#include <QImage>
#include <QVariant>

class RobotController : public QObject {

    Q_OBJECT

public:

    RobotController(QQmlApplicationEngine *engine, RobotsController* parent, QString ip, QString robotName);
    ~RobotController();

    /**
     * @brief stopThreads
     * Stop all the workers and threads
     */
    void stopThreads(void);

    /**
     * @brief sendCommand
     * @param cmd
     * Send a command to the command controller
     */
    void sendCommand(const QString cmd);

    /**
     * @brief sendNewMap
     * @param mapId
     * @param date
     * @param mapMetadata
     * @param mapImage
     * Send the new map to the newMapWorker
     */
    void sendNewMap(const QString mapId, const QString date, const QString mapMetadata, const QImage &mapImage);

    void sendMP3(const QString fileName, const bool isLastMP3File);

    /**
     * @brief ping
     * End a ping to the robotsController to tell that the robot is still alive
     */
    void ping(void);

    /**
     * @brief sendTeleop
     * @param teleop
     * Send the teleop command to the teleop worker
     */
    void sendTeleop(const int teleop);

    /**
     * @brief clearObstacles
     * To remove all the obstacles when we stop using the laser
     */
    void clearObstacles(bool activated);
    /**
     * @brief updateRobotPosition
     * @param x
     * @param y
     * @param orientation
     * To update the position of the painted item and propagate these properties to the qml side
     */
    void updateRobotPosition(double x, double y, double orientation);

    /**
     * @brief getRebootRequestFromRobotsController
     * @param ip
     * waiting for the robots controller to request a reboot order
     */
    void getRebootRequestFromRobotsController();

private:
    /**
     * @brief launchWorkers
     * @param mainWindow
     * connects the workers in order to receive map, robot position and laser data
     */
    void launchWorkers(void);

private slots:
    /**
     * @brief mapReceivedSlot
     * @param mapArray
     * @param who
     * @param mapId
     * @param mapDate
     * @param resolution
     * @param originX
     * @param originY
     * @param map_width
     * @param map_height
     * We just received a map from the robot, <who> tells us what the map is for (map to use, scanning, merging, recovering)
     */
    void mapReceivedSlot(const QByteArray mapArray, const int who, const QString mapId, const QString mapDate, const QString resolution, const QString originX, const QString originY, const int map_width, const int map_height);

    /**
     * @brief doneSendingMapSlot
     * Finished to send the map to the robot
     */
    void doneSendingMapSlot(bool deleteHomePath);

    /**
     * @brief updateRobot
     * @param posX
     * @param posY
     * @param ori
     * Just received the position of the robot
     */
    void updateRobot(const double posX, const double posY, const double ori);

    /**
     * @brief robotIsDeadSlot
     * The robot disconnected
     */
    void robotIsDeadSlot(void);

    /**
     * @brief updateRobotInfoSlot
     * @param robotInfo
     * The robot just connected so we update all its info
     */
    void updateRobotInfoSlot(const QString robotInfo);

    /**
     * @brief connectedSlot
     * We are connected to the robot so we start all the workers/threads
     */
    void connectedSlot(void);

    /**
     * @brief updateObstacles
     * @param angle_min
     * @param angle_max
     * @param angle_increment
     * @param ranges
     * To update the display of the obstacles on the map when we receive it from the robot
     */
    void updateObstacles(float angle_min, float angle_max, float angle_increment, QVector<float> ranges);

signals:
    /**
     * @brief robotIsDead
     * @param ip
     * Tell the robotsController that the rbot is dead
     */
    void robotIsDead(QString ip);

    /**
     * @brief pingSignal
     * Tell the cmdRobotWorker that the robot is still alive
     */
    void pingSignal(void);

    /**
     * @brief sendNewMapSignal
     * @param mapId
     * @param date
     * @param metadata
     * @param map
     * Tell the newMapWorker to send the new map
     */
    void sendNewMapSignal(QString mapId, QString date, QString metadata, QImage map);

//    void sendMP3Signal(QString fileName);

    /**
     * @brief teleopCmd
     * Send the teleop command to the worker
     */
    void teleopCmd(int);

    /**
     * @brief newRobotPos
     * @param ip
     * @param posX
     * @param posY
     * @param ori
     * Send the new position of the robot to the robotsController
     */
    void newRobotPos(QString ip, double posX, double posY, double ori);

    /**
     * @brief newMapFromRobot
     * @param ip
     * @param mapArray
     * @param mapId
     * @param mapDate
     * @param resolution
     * @param originX
     * @param originY
     * @param map_width
     * @param map_height
     * Send the new map to the robotsController to be used as the main one
     */
    void newMapFromRobot(QString ip, QByteArray mapArray, QString mapId, QString mapDate, QString resolution, QString originX, QString originY, int map_width, int map_height);

    /**
     * @brief mapToMergeFromRobot
     * @param mapArray
     * @param resolution
     * Send the new map to the robotsController to be used in the merge window
     */

    /**
     * @brief receivedScanMap
     * @param ip
     * @param mapArray
     * @param resolution
     * @param originX
     * @param originY
     * @param map_width
     * @param map_height
     * Send the new map to the robotsController to be used in the scan window
     */
    void receivedScanMap(QString ip, QByteArray mapArray, QString resolution, QString originX, QString originY, int map_width, int map_height);
    /**
     * @brief receivedMap
     * @param ip
     * emitted when a new map for merge is received so that the model can be updated on the qml side
     */
    void receivedMap(QVariant ip);

    /**
     * @brief resetHomePath
     * @param ip
     * To reset both the path and home of the robot at ip <ip>
     */
    void resetHomePath(QString ip);

    /**
     * @brief updateRobotInfo
     * @param ip
     * @param robotInfo
     * Tell the robotsController to update the new robot info
     */
    void updateRobotInfo(QString ip, QString robotInfo);

    /// Tell the workers to stop their connection to the robot
    void stopCmdRobotWorker(void);
    void stopRobotWorker(void);
    void stopNewMapWorker(void);
    void stopLaserWorker(void);
    void stopMapWorker(void);
    void stopTeleopWorker(void);

    /// Tell the workers to start their connection to the robot
    void startCmdRobotWorker(void);
    void startRobotWorker(void);
    void startNewMapWorker(void);
    void startLaserWorker(void);
    void startMapWorker(void);
    void startTeleopWorker(void);

private:
    QString ip;
    QString robotInfo_;
    bool sendingMap;
    QPointer<CommandController> commandController;

    QPointer<CmdRobotWorker> cmdRobotWorker;
    QPointer<CmdRobotWorker> cmdMP3Worker;
    QPointer<RobotPositionWorker> robotWorker;
    QPointer<SendNewMapWorker> newMapWorker;
    QPointer<SendNewMapWorker> newMP3Worker;
//    QPointer<LaserWorker> laserWorker;
    QPointer<ScanMapWorker> mapWorker;
    QPointer<TeleopWorker> teleopWorker;

    QThread cmdThread;
    QThread robotThread;
    QThread newMapThread;
//    QThread laserThread;
    QThread mapThread;
    QThread teleopThread;

    ObstaclesPaintedItem* paintedItem;
};

#endif /// ROBOTCONTROLLER_H
