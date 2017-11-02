#ifndef PATHPOINT_H
#define PATHPOINT_H

class Point;

#include <QObject>
#include <QPointer>

class PathPoint : public QObject {

public:

    PathPoint(const QString _name, const double _x, const double _y, const int _waitTime, const int _orientation, QObject *parent);

    QPointer<Point> getPoint(void) const { return point; }
    int getWaitTime(void) const { return waitTime; }

    void setWaitTime(const int _waitTime) { waitTime = _waitTime; }

private:
    /**
     * @brief waitTime
     * How long the robot needs to wait if its action is to wait
     */
    int waitTime;

    /**
     * @brief point
     * The point where the robot goes
     */
    QPointer<Point> point;
};

#endif /// PATHPOINT_H
