#include "point.h"
#include "Model/Points/points.h"
#include <QDebug>
#include <QDataStream>
#include <iostream>

Point::Point(void): name(""), position(Position(0.0, 0.0)), type(PERM) {}

Point::Point(const QString name, const double x, const double y, const PointType _type):
    name(name), position(Position(x, y)), type(_type)
{}

Point::Point(const QString name, const Position position, const PointType _type) : name(name), position(position), type(_type), robotName("") {}

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
            Q_UNREACHABLE();
        break;
    }
}

std::ostream& operator <<(std::ostream& stream, const Point& point){
    point.display(stream);
    return stream;
}

/// serializing
QDataStream& operator<<(QDataStream& out, const Point& point){
    out << point.getName() << point.getPosition().getX() << point.getPosition().getY() << static_cast<qint32>(point.getType());
    return out;
}

/// deserializing
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
    return (!point.getName().compare(this->getName())) && this->getPosition() == point.getPosition();
}

/// floating comparisons not being reliable we arbitrarily decided that two points had the same pos if
/// both their x and y coordinates were less than 0.01 appart
 bool Point::comparePos(const double x, const double y) const {
     return (abs(x - this->getPosition().getX()) < 0.01 &&
             abs(y - this->getPosition().getY()) < 0.01);
 }

 bool Point::comparePos(const Position pos) const {
     return (abs(pos.getX() - this->getPosition().getX()) < 0.01 &&
             abs(pos.getY() - this->getPosition().getY()) < 0.01);
 }

 bool Point::setHome(const PointType _type){
    if(_type != HOME)
        type = _type;
    else if((_type == HOME) && (type != HOME)){
        type = _type;
        if (!name.compare(TMP_POINT_NAME))
            name = QString::number(position.getX(),'f', 1) + "; " + QString::number(position.getY(), 'f', 1);
        return true;
    }
    return false;
 }
