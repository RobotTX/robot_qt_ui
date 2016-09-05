#ifndef ROBOTS_H
#define ROBOTS_H

class RobotView;
class Point;

#include <QVector>
#include <QMap>
#include <QSharedPointer>

/**
 * @brief The Robots class
 * Represent all the robots currently used by the app
 */
class Robots{
public:
    Robots();

    /// Getters
    QVector<RobotView*> getRobotsVector() const { return robotsVector; }
    QMap<QString, QString> getRobotsNameMap() const {return robotsNameMap;}

    /// Setters
    void setRobotsVector(const QVector<RobotView*>& _robotsVector) { robotsVector = _robotsVector; }
    void setRobotsNameMap(const QMap<QString, QString> &_robotsNameMap) {robotsNameMap = _robotsNameMap;}

    /**
     * @brief add
     * @param robot
     * Add the robot in param to the Vector of robots
     */
    void add(RobotView* const robot);

    /**
     * @brief remove
     * @param robot
     * Remove the robot in param from the Vector of robots
     */
    void remove(RobotView *robot);

    /**
     * @brief removeByName
     * @param name
     * Remove a robot using its name (which should be unique)
     */
    void removeByName(const QString name);

    /**
     * @brief removeByIp
     * @param ip
     * Same as removeByName but using the robot ip
     */
    void removeByIp(const QString ip);

    /**
     * @brief getRobotViewByName
     * @param name
     * @return
     * Get a robotView from the name of its robot
     */
    RobotView* getRobotViewByName(const QString name) const;

    /**
     * @brief getRobotViewByIp
     * @param name
     * @return
     * Same as above but with the ip
     * Gets a robotView from the ip of its robot
     */
    RobotView* getRobotViewByIp(const QString ip) const;

    /**
     * @brief setSelected
     * @param robotView
     * Setz which robot is currently selected, to highlight it and know
     * on which robot some functions need to be applied
     */
    void setSelected(RobotView* const robotView);

    /**
     * @brief existRobotName
     * @param name
     * @return
     * Checks if a robot with the given name already exists
     */
    bool existRobotName(const QString name) const;

    /**
     * @brief getRobotId
     * @param name
     * @return int
     * returns the index of the robot whose name is passed as a parameter within the vector of robots
     */
    int getRobotId(const QString name) const;

    /**
     * @brief findRobotUsingHome
     * @param name
     * @return returns the RobotView whose home point's name is passed as an argument
     */
    RobotView* findRobotUsingHome(const QString name) const;

    /**
     * @brief findRobotUsingTmpPointInPath
     * @param point
     * @return the RobotView using the given temporary point in its path
     */
    RobotView* findRobotUsingTmpPointInPath(const QSharedPointer<Point> point) const;

    /**
     * @brief insertRobotsNameMap
     * @param ip
     * @param name
     * Insert the given robot name with it's ip in robotsNameMap
     */
    void insertRobotsNameMap(QString ip, QString name) {robotsNameMap.insert(ip, name);}

    /**
     * @brief deselect
     * Deselect any selected robotView
     */
    void deselect(void);

    /**
     * @brief hide
     * Hide all robots
     */
    void hide(void) const;

    /**
     * @brief show
     * Show all robots
     */
    void show(void) const;

    /**
     * @brief showRobot
     * @param id
     * Show the robot at the given id
     */
    void showRobot(const int id) const;

private:

    QVector<RobotView*> robotsVector;

    /**
     * @brief robotsNameMap
     * A map of the robot
     */
    QMap<QString, QString> robotsNameMap;
};

#endif // ROBOTS_H
