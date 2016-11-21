#include "pathpoint.h"
#include <QDebug>


PathPoint::PathPoint(const Point &_point, const int _waitTime): point(_point), waitTime(_waitTime) {}

/// the main purpose of this constructor is the deserialization of a path point
PathPoint::PathPoint(void): point(Point("DefaultPoint", -1.0, -1.0)), waitTime(0) {}

QDataStream& operator>>(QDataStream& in, PathPoint& pathPoint){
    Point _point;
    qint32 _waitTime;
    in >> _point >> _waitTime;
    pathPoint.setPoint(_point);
    pathPoint.setWaitTime(static_cast<int>(_waitTime));
    return in;
}

QDataStream& operator<<(QDataStream& out, const PathPoint& pathPoint){
    out << pathPoint.getPoint() << static_cast<qint32>(pathPoint.getWaitTime());
    return out;
}

bool operator==(const PathPoint& pathPoint, const PathPoint& otherPathPoint){
    return pathPoint.getPoint().getPosition() == otherPathPoint.getPoint().getPosition() &&
            pathPoint.getWaitTime() == otherPathPoint.getWaitTime();
}

bool operator!=(const PathPoint& pathPoint, const PathPoint& otherPathPoint){
    return !(pathPoint == otherPathPoint);
}

std::ostream& operator <<(std::ostream& stream, const PathPoint& point){
    point.display(stream);
    return stream;
}

void PathPoint::display(std::ostream& stream) const {
    stream << point.getPosition().getX() << " " << point.getPosition().getY() << " " << waitTime;
}


