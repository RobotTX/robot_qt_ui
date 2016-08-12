#include "pathpoint.h"
#include <QDebug>


PathPoint::PathPoint(const Point &_point, const Action &_action, const int _waitTime): point(_point), action(_action), waitTime(_waitTime)
{
}

QDataStream& operator>>(QDataStream& in, PathPoint& pathPoint){
    Point _point;
    qint32 _waitTime;
    in >> _point >> _waitTime;
    qDebug() << "Deserializing" << _point.getName() << _point.getPosition().getX() << _point.getPosition().getY() << _waitTime;
    pathPoint.setPoint(_point);
    pathPoint.setWaitTime(static_cast<int>(_waitTime));
    return in;
}

QDataStream& operator<<(QDataStream& out, const PathPoint& pathPoint){
    qDebug() << "serializing pathpoint" << pathPoint.getPoint().getName() << pathPoint.getPoint().getPosition().getX() <<
                pathPoint.getPoint().getPosition().getY() << pathPoint.getWaitTime();
    out << pathPoint.getPoint() << static_cast<qint32>(pathPoint.getWaitTime());
    return out;
}
