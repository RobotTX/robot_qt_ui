#include "point.h"
#include <QDebug>
#include <QDataStream>

Point::Point(void): name(""), position(Position(0.0, 0.0)), permanent(true)
{}

Point::Point(const QString name, const double x, const double y, const bool permanent):
    name(name), position(Position(x, y)), permanent(permanent) {}

Point::Point(const QString name, const Position position, const bool _displayed, const bool _permanent) : name(name), position(position), displayed(_displayed), permanent(_permanent)
{}

void Point::display(std::ostream& stream) const {
    stream << name.toStdString() << " (" << position.getX() << ", " << position.getY() << ")";
}

std::ostream& operator <<(std::ostream& stream, const Point& point){
    point.display(stream);
    return stream;
}

QDataStream& operator<<(QDataStream& out, const Point& point){
    out << point.getName() << point.getPosition().getX() << point.getPosition().getY() << point.isPermanent();
    return out;
}

QDataStream& operator>>(QDataStream& in, Point& point){
    QString name;
    double x;
    double y;
    bool permanent;
    in >> name >> x >> y >> permanent;
    point = Point(name, x, y, permanent);
    return in;
}

bool Point::operator==(const Point& point) const {
    if(!point.getName().compare(this->getName()) &&
            /// because it's a bad practice to directly compare double values we just decide of a range of coordinates that are close enough from each other to be considered the same
            abs(point.getPosition().getX() - this->getPosition().getX()) < 0.01 &&
            abs(point.getPosition().getY() - this->getPosition().getY()) < 0.01)
        return true;
    else
        return false;
}

 bool Point::comparePos(double x, double y) const {
     if(abs(x - this->getPosition().getX()) < 0.01 &&
             abs(y - this->getPosition().getY()) < 0.01)
         return true;
     else
         return false;
 }
