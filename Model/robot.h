#ifndef ROBOT_H
#define ROBOT_H

class PathPoint;
class CmdRobotThread;
class QMainWindow;
class ScanRobotThread;
class ScanMetadataThread;
class SendNewMapThread;

#include "Model/position.h"
#include <QString>
#include <QVector>
#include <QSharedPointer>
#include <QtNetwork/QTcpSocket>
#include <QUuid>
#include <QDataStream>
#include "View/pointview.h"

#define PORT_MAP_METADATA 4000
#define PORT_ROBOT_POS 4001
#define PORT_MAP 4002
#define PORT_CMD 5600
#define PORT_NEW_MAP 5601

/**
 * @brief The Robot class
 * Represent a Robot
 */

class Robot : public QObject{
    Q_OBJECT
public:
    Robot(const QString name, const QString addressIp);
    Robot();
    ~Robot();

    /// Getters
    Position getPosition(void) const { return position; }
    float getOrientation(void) const { return orientation; }
    QString getName(void) const { return name; }
    QString getIp(void) const { return ip; }
    unsigned int getBatteryLevel(void) const { return batteryLevel; }
    QString getWifi(void) const { return wifi; }
    QSharedPointer<PointView> getHome(void) const { return home; }
    QVector<QSharedPointer<PathPoint>> getPath(void) const { return path; }
    bool isPlayingPath(void) const { return playingPath; }

    /// Setters
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
    void setMapId(const QUuid _mapId) { mapId = _mapId; }
    QUuid getMapId(void) const { return mapId; }

    /**
     * @brief display
     * @param stream
     * Displays the robot information
     */
    void display(std::ostream& stream) const;

    /**
     * @brief sendCommand
     * @param cmd
     * @return
     * Called to send a command to the robot
     */
    bool sendCommand(const QString cmd);
    void sendNewMap(QByteArray cmd);

    QString waitAnswer();
    void resetCommandAnswer();
    void stopThreads();
    void ping();
    void clearPath();

signals:
    void pingSignal();
    void sendCommandSignal(QString cmd);
    void sendNewMapSignal(QByteArray cmd);

private slots:
    void doneSendingNewMapSlot();

private:
    QString name;
    QString ip;
    Position position;
    float orientation;
    unsigned int batteryLevel;
    QString wifi;

    CmdRobotThread* cmdThread;
    /**
     * @brief home
     * Home point of the robot
     */
    QSharedPointer<PointView> home;

    /**
     * @brief path
     * Path linked to that robot
     */
    QVector<QSharedPointer<PathPoint>> path;

    /**
     * @brief playingPath
     * Boolean representing whether or not the robot is currently executing the path
     */
    bool playingPath;
    ScanRobotThread* robotThread;
    ScanMetadataThread* metadataThread;
    SendNewMapThread* newMapThread;
    QUuid mapId;
    bool sendingMap;
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

#endif // ROBOT_H
