#ifndef ROBOTS_H
#define ROBOTS_H

class RobotView;

#include <QVector>

/**
 * @brief The Robots class
 * Represent all the robots currently used by the app
 */
class Robots{
public:
    Robots();
    ~Robots();

    /**
     * @brief setRobotsVector
     * @param _robotsVector
     * Setter for the vector of RobotView
     */
    void setRobotsVector(const QVector<RobotView*>& _robotsVector) { robotsVector = _robotsVector; }

    /**
     * @brief getRobotsVector
     * @return A vector of RobotView
     */
    QVector<RobotView*> getRobotsVector() const { return robotsVector; }

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
    RobotView* getRobotViewByName(const QString name);

    /**
     * @brief getRobotViewByIp
     * @param name
     * @return
     * Same as above but with the ip
     * Gets a robotView from the ip of its robot
     */
    RobotView* getRobotViewByIp(const QString ip);

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
    bool existRobotName(const QString name);


    int getRobotId(const QString name);

    RobotView* findRobotUsingHome(const QString name) const;

private:
    /**
     * @brief robotsVector
     * The vector of RobotView (which also contains the robot)
     */
    QVector<RobotView*> robotsVector;
};

#endif // ROBOTS_H
