#ifndef PATHPOINT_H
#define PATHPOINT_H

#include "Model/point.h"

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

    /// Getters
    Point getPoint(void) { return point; }
    Action getAction(void) { return action; }
    int getWaitTime(void) { return waitTime; }

    /// Setters
    void setPoint(const Point& _point) { point = _point; }
    void setAction(const Action& _action) { action = _action; }
    void setWaitTime(const int _waitTime) { waitTime = _waitTime; }

private:
    /**
     * @brief point
     * The point where the robot go
     */
    Point point;

    /**
     * @brief action
     * The action the robot need to do after ariving to the point
     */
    Action action;

    /**
     * @brief waitTime
     * How long the robot need to wait if its action is to wait
     */
    int waitTime;
};

#endif // PATHPOINT_H

