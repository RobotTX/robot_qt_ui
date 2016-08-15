#include "point.h"
#include <QDebug>
#include <QDataStream>
#include <iostream>

Point::Point(void): name(""), position(Position(0.0, 0.0)), type(PERM){
}

Point::Point(const QString name, const double x, const double y, const PointType _type):
    name(name), position(Position(x, y)), type(_type) {
}

Point::Point(const QString name, const Position position, const PointType _type) : name(name), position(position), type(_type){
}

void Point::display(std::ostream& stream) const {
    stream << name.toStdString() << " (" << position.getX() << ", " << position.getY() << ") ";
    switch(type){
        case PERM:
            stream << "which is Permanent";
        break;
        case TEMP:
            stream << "which is Temporary";
        break;
        case HOME:
            stream << "which is a Robot Home";
        break;
        case PATH:
            stream << "which is a Path Point";
        break;
        default:
            qDebug() << "Point::display : you should not be here, you probably did not implement the behavior for the type" << type;
        break;
    }
}

std::ostream& operator <<(std::ostream& stream, const Point& point){
    point.display(stream);
    return stream;
}

QDataStream& operator<<(QDataStream& out, const Point& point){
    out << point.getName() << point.getPosition().getX() << point.getPosition().getY() << static_cast<qint32>(point.getType());
    return out;
}

QDataStream& operator>>(QDataStream& in, Point& point){
    QString name;
    double x;
    double y;
    qint32 typeInt;
    in >> name >> x >> y >> typeInt;
    Point::PointType type = static_cast<Point::PointType>(typeInt);
    point = Point(name, x, y, type);
    return in;
}

bool Point::operator==(const Point& point) const {
    if(!point.getName().compare(this->getName()) &&
            /// because it's a bad practice to directly compare double values (could lead to unexpected behaviors) we just decide of a range of coordinates that are close enough from each other to be considered the same
            abs(point.getPosition().getX() - this->getPosition().getX()) < 0.01 &&
            abs(point.getPosition().getY() - this->getPosition().getY()) < 0.01)
        return true;
    else
        return false;
}

 bool Point::comparePos(const double x, const double y) const {
     if(abs(x - this->getPosition().getX()) < 0.01 &&
             abs(y - this->getPosition().getY()) < 0.01)
         return true;
     else
         return false;
 }

 bool Point::comparePos(const Position pos) const {
     if(abs(pos.getX() - this->getPosition().getX()) < 0.01 &&
             abs(pos.getY() - this->getPosition().getY()) < 0.01)
         return true;
     else
         return false;
 }

 bool Point::setHome(const PointType _type, const QString robotName){
    if(_type != HOME){
        type = _type;
        name = QString::number(position.getX(),'f', 1) + "; " + QString::number(position.getY(),'f', 1);
    } else if((_type == HOME) && (type != HOME)){
        type = _type;
        if (name == "tmpPoint")
            name = QString::number(position.getX(),'f', 1) + "; " + QString::number(position.getY(),'f', 1);
        return true;
    }
    return false;
 }
