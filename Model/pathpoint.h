#ifndef PATHPOINT_H
#define PATHPOINT_H

#include "Model/point.h"

#include <QDataStream>

/**
 * @brief The PathPoint class
 * Represent a point on a robot path
 */
class PathPoint {
public:
    /**
     * @brief The Action enum
     * Used to know if we should wait for a certain amount of time
     * or for a human action when the robot reach the point
     */
    PathPoint(const Point& point, const int waitTime = 0);
    PathPoint(void);

    Point getPoint(void) const { return point; }
    int getWaitTime(void) const { return waitTime; }

    void setPoint(const Point& _point) { point = _point; }
    void setWaitTime(const int _waitTime) { waitTime = _waitTime; }

    void display(std::ostream& stream) const;

private:
    /**
     * @brief point
     * The point where the robot goes
     */
    Point point;

    /**
     * @brief waitTime
     * How long the robot needs to wait if its action is to wait
     */
    int waitTime;


};

/**
 * @brief operator <<
 * @param out
 * @param robot
 * @return
 * Serialization of a pathpoint
 */
QDataStream& operator<<(QDataStream& out, const PathPoint& robot);

/**
 * @brief operator >>
 * @param in
 * @param robot
 * @return
 * Deserialization of a pathpoint
 */
QDataStream& operator>>(QDataStream& in, PathPoint& robot);

bool operator==(const PathPoint& pathPoint, const PathPoint& otherPathPoint);

bool operator!=(const PathPoint& pathPoint, const PathPoint& otherPathPoint);

std::ostream& operator <<(std::ostream& stream, const PathPoint& point);

#endif /// PATHPOINT_H

