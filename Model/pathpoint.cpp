#include "pathpoint.h"
#include <QDebug>


PathPoint::PathPoint(const Point &_point, const Action &_action, const int _waitTime): point(_point), action(_action), waitTime(_waitTime)
{
}

/// the main purpose of this constructor is the deserialization of a path point
PathPoint::PathPoint(void): point(Point("DefaultPoint", -1.0, -1.0)), action(Action::HUMAN_ACTION), waitTime(0) {}

QDataStream& operator>>(QDataStream& in, PathPoint& pathPoint){
    Point _point;
    qint32 _action;
    qint32 _waitTime;
    in >> _point >> _action >> _waitTime;
    pathPoint.setPoint(_point);
    pathPoint.setAction(static_cast<PathPoint::Action>(_action));
    pathPoint.setWaitTime(static_cast<int>(_waitTime));
    return in;
}

QDataStream& operator<<(QDataStream& out, const PathPoint& pathPoint){
    out << pathPoint.getPoint() << static_cast<qint32>(pathPoint.getAction()) << static_cast<qint32>(pathPoint.getWaitTime());
    return out;
}
