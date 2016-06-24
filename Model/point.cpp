#include "point.h"
#include <QDebug>
#include <QDataStream>
#include <iostream>

Point::Point(void): name(""), position(Position(0.0, 0.0)), permanent(true), home(false){
}

Point::Point(const QString name, const double x, const double y, const bool _permanent):
    name(name), position(Position(x, y)), permanent(_permanent), home(false) {
}

Point::Point(const QString _name, const double x, const double y, const bool _displayed, const bool _permanent):
    name(_name), position(Position(x, y)), displayed(_displayed), permanent(_permanent)
{}

Point::Point(const QString name, const Position position, const bool _displayed, const bool _permanent) : name(name), position(position), displayed(_displayed), permanent(_permanent), home(false){
}

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

 bool Point::setHome(const bool _home, const QString robotName){
    if(!_home){
        home = _home;
        name = QString::number(position.getX(),'f', 1) + "; " + QString::number(position.getY(),'f', 1);
    } else if(_home && !home){
        home = _home;
        name = "Home_" + robotName;
        return true;
    }
    return false;
 }
