#ifndef PATHPOINT_H
#define PATHPOINT_H

#include "Model/point.h"

#include <QDataStream>

/**
 * @brief The PathPoint class
 * Represent a point on a robot path
 */
class PathPoint{
public:
    /**
     * @brief The Action enum
     * Used to know if we should wait for a certain amount of time
     * or for a human action when the robot reach the point
     */
    enum Action { WAIT, HUMAN_ACTION };
    PathPoint(const Point& point, const Action& action, const int waitTime = 0);
    PathPoint(void);

    /// Getters
    Point getPoint(void) const { return point; }
    Action getAction(void) const { return action; }
    int getWaitTime(void) const { return waitTime; }

    /// Setters
    void setPoint(const Point& _point) { point = _point; }
    void setAction(const Action& _action) { action = _action; }
    void setWaitTime(const int _waitTime) { waitTime = _waitTime; }

private:
    /**
     * @brief point
     * The point where the robot goes
     */
    Point point;

    /**
     * @brief action
     * The action the robot needs to do after arriving to the point
     */
    Action action;

    /**
     * @brief waitTime
     * How long the robot needs to wait if its action is to wait
     */
    int waitTime;


};

/**
 * @brief operator <<
 * @param out
 * @param pathPoint
 * @return
 */
QDataStream& operator<<(QDataStream& out, const PathPoint& robot);
QDataStream& operator>>(QDataStream& in, PathPoint& robot);

#endif // PATHPOINT_H

