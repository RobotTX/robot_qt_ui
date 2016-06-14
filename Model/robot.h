#ifndef ROBOT_H
#define ROBOT_H

class PathPoint;
class Point;
class CmdRobotThread;
class QMainWindow;

#include "Model/position.h"
#include <QString>
#include <QVector>

/**
 * @brief The Robot class
 * Represent a Robot
 */

class Robot{
public:
    Robot(const QString name, const QString addressIp, const int port, QMainWindow* parent);
    Robot();
    ~Robot();

    /// Getters
    Position getPosition() const { return position; }
    float getOrientation() const { return orientation; }
    QString getName() const { return name; }
    QString getIp() const { return ip; }
    unsigned int getBatteryLevel(void) const { return batteryLevel; }
    QString getWifi() const { return wifi; }
    Point* getHome() { return home; }
    QVector<PathPoint*> getPath() { return path; }
    bool isPlayingPath() { return playingPath; }

    /// Setters
    void setPlayingPath( bool playPath) { playingPath = playPath; }
    void setPath( QVector<PathPoint*> _path) { path = _path; }
    void setHome(Point* _home) { home = _home; }
    void setWifi(const QString _wifi) { wifi = _wifi; }
    void setIp(const QString _ip) { ip = _ip; }
    void setBatteryLevel(const unsigned int _batteryLevel) { batteryLevel = _batteryLevel; }
    void setName(const QString _name) { name = _name; }
    void setOrientation(const float _orientation) { orientation = _orientation; }
    void setPosition(const Position _position) { position = _position; }
    void setPosition(const float _x, const float _y) { position = Position(_x,_y); }

    /**
     * @brief display
     * @param stream
     * Display the robot informations
     */
    void display(std::ostream& stream) const;

    /**
     * @brief sendCommand
     * @param cmd
     * @return
     * Called to send a command to the robot
     */
    bool sendCommand(QString cmd);


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
    Point* home;

    /**
     * @brief path
     * Path linked to that robot
     */
    QVector<PathPoint*> path;

    /**
     * @brief playingPath
     * Boolean representing whether or not the robot is currently executing the path
     */
    bool playingPath;
};

/**
 * @brief operator <<
 * @param stream
 * @param robot
 * @return
 * Overload of the << operator for cout
 */
std::ostream& operator <<(std::ostream& stream, Robot const& robot);

#endif // ROBOT_H
