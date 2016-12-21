#ifndef ROBOT_H
#define ROBOT_H

class PathPoint;
class QMainWindow;
class Map;

#include "Model/position.h"
#include <QString>
#include <QVector>
#include <QSharedPointer>
#include <QtNetwork/QTcpSocket>
#include <QUuid>
#include <QDataStream>
#include "View/pointview.h"
#include "Model/paths.h"
#include <QThread>
#include "Controller/cmdrobotworker.h"
#include "Controller/robotpositionworker.h"
#include "Controller/metadataworker.h"
#include "Controller/sendnewmapworker.h"
#include "Controller/localmapworker.h"
#include "Controller/scanmapworker.h"

#define PORT_MAP_METADATA 4000
#define PORT_ROBOT_POS 4001
#define PORT_MAP 4002
#define PORT_CMD 5600
#define PORT_NEW_MAP 5601
#define PORT_LOCAL_MAP 5605

/**
 * @brief The Robot class
 * Represent a Robot
 */

class Robot : public QObject {
    Q_OBJECT
public:
    Robot(MainWindow* mainWindow, const QSharedPointer<Paths> &_paths, const QString name, const QString addressIp);
    Robot();
    ~Robot();

    Position getPosition(void) const { return position; }
    float getOrientation(void) const { return orientation; }
    QString getName(void) const { return name; }
    QString getIp(void) const { return ip; }
    int getBatteryLevel(void) const { return batteryLevel; }
    QString getWifi(void) const { return wifi; }
    QSharedPointer<PointView> getHome(void) const { return home; }
    QVector<QSharedPointer<PathPoint>> getPath(void) const { return path; }
    bool isPlayingPath(void) const { return playingPath; }
    QString getPathName(void) const { return pathName; }
    QString getGroupPathName(void) const { return groupName; }
    QSharedPointer<Paths> getPaths(void) const { return paths; }
    bool isSendingMap(void) const { return sendingMap; }

    void setPlayingPath(const bool playPath) { playingPath = playPath; }
    void setPath(const QVector<QSharedPointer<PathPoint>>& _path) { path = _path; }
    void setHome(QSharedPointer<PointView> _home) { home = _home; }
    void setWifi(const QString _wifi) { wifi = _wifi; }
    void setIp(const QString _ip) { ip = _ip; }
    void setBatteryLevel(const unsigned int _batteryLevel) { batteryLevel = _batteryLevel; }
    void setName(const QString _name) { name = _name; }
    void setOrientation(const float _orientation) { orientation = _orientation; }
    void setPosition(const Position _position) { position = _position; }
    void setPosition(const float _x, const float _y) { position = Position(_x,_y); }
    void setPathName(const QString name) { pathName = name; }
    void setGroupPathName(const QString name) { groupName = name; }

    /**
     * @brief launchWorkers
     * @param mainWindow
     * connects the workers in order to receive map, robot position and laser data
     */
    void launchWorkers(MainWindow* mainWindow);

     /**
     * @brief display
     * @param stream
     * Displays the robot information
     */
    void display(std::ostream& stream) const;

    /**
     * @brief sendCommand
     * @param cmd
     * Called to send a command to the robot
     */
    void sendCommand(const QString cmd);

    /**
     * @brief sendNewMap
     * @param cmd
     * To send the new map to the robot
     */
    void sendNewMap(QSharedPointer<Map> map);

    /**
     * @brief stopThreads
     * Stop every threads
     */
    void stopThreads();

    /**
     * @brief ping
     * To tell the command thread of this robot that we are still alive
     */
    void ping();

    /**
     * @brief clearPath
     * Delete the path of this robot
     */
    void clearPath();

signals:
    /**
     * @brief pingSignal
     * When a robot connect to the update socket in mainwindow, we tell the command thread that the robot is still alive
     */
    void pingSignal();

    /**
     * @brief sendCommandSignal
     * @param cmd
     * To tell to the command thread to send a command
     */
    void sendCommandSignal(QString cmd);

    /**
     * @brief sendNewMapSignal
     * @param mapId
     * @param date
     * @param metadata
     * @param map
     * To tell the map thread to send a new map to the robot
     */
    void sendNewMapSignal(QString mapId, QString date, QString metadata, QImage map);

    void stopCmdRobotWorker();
    void stopRobotWorker();
    void stopMetadataWorker();
    void stopNewMapWorker();
    void stopLocalMapWorker();
    void stopMapWorker();

    void startCmdRobotWorker();
    void startRobotWorker();
    void startMetadataWorker();
    void startNewMapWorker();
    void startLocalMapWorker();
    void startMapWorker();

private slots:
    void doneSendingMapSlot();
    void portSentSlot();

private:
    QSharedPointer<Paths> paths;
    QString name;
    QString ip;
    Position position;
    float orientation;
    int batteryLevel;
    QString wifi;
    QSharedPointer<PointView> home;
    QVector<QSharedPointer<PathPoint>> path;
    bool playingPath;
    QString pathName;
    QString groupName;
    bool sendingMap;

    QPointer<CmdRobotWorker> cmdRobotWorker;
    QPointer<RobotPositionWorker> robotWorker;
    QPointer<MetadataWorker> metadataWorker;
    QPointer<SendNewMapWorker> newMapWorker;
    QPointer<LocalMapWorker> localMapWorker;
    QPointer<ScanMapWorker> mapWorker;

    QThread cmdThread;
    QThread robotThread;
    QThread metadataThread;
    QThread newMapThread;
    QThread localMapThread;
    QThread mapThread;
};

/**
 * @brief operator <<
 * @param out
 * @param robot
 * @return
 */
QDataStream& operator<<(QDataStream& out, const Robot& robot);
QDataStream& operator>>(QDataStream& in, Robot& robot);

/**
 * @brief operator <<
 * @param stream
 * @param robot
 * @return
 * Overload of the << operator for cout
 */
std::ostream& operator <<(std::ostream& stream, Robot const& robot);

#endif /// ROBOT_H
