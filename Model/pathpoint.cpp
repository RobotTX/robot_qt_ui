#include "pathpoint.h"
#include <QDebug>


PathPoint::PathPoint(const Point &_point, const Action &_action, const int _waitTime): point(_point), action(_action), waitTime(_waitTime)
{
}

QDataStream& operator>>(QDataStream& in, PathPoint& pathPoint){
    qDebug() << "PathPoint operator >> called";
    Point _point;
    qint32 _waitTime;
    in >> _point >> _waitTime;
    pathPoint.setPoint(_point);
    pathPoint.setWaitTime(_waitTime);
    return in;
}

QDataStream& operator<<(QDataStream& out, const PathPoint& pathPoint){
    qDebug() << "PathPoint operator << called";
    out << pathPoint.getPoint() << pathPoint.getWaitTime();
    return out;
}
