#ifndef PATHPOINT_H
#define PATHPOINT_H

class Point;

#include <QObject>

class PathPoint : public QObject {
public:
    PathPoint(const QString _name, const double _x, const double _y, const int _waitTime, QObject *parent);

    Point* getPoint(void) const { return point; }
    int getWaitTime(void) const { return waitTime; }

    void setWaitTime(const int _waitTime) { waitTime = _waitTime; }


private:
    /**
     * @brief point
     * The point where the robot goes
     */
    Point* point;

    /**
     * @brief waitTime
     * How long the robot needs to wait if its action is to wait
     */
    int waitTime;
};

#endif // PATHPOINT_H
