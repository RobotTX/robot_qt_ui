#ifndef POINT_H
#define POINT_H

class QDataStream;

#define TMP_POINT_NAME "tmpPoint"
#define PATH_POINT_NAME "PathPoint"

#include "Model/position.h"
#include <QString>

/**
 * @brief The Point class
 * This class provides a model for a point
 * A point is identified by a name that is unique, a Position objet whose class is described below and a boolean attribute
 * to distinguish permanent and temporary points
 */

class Point {

public:
    enum PointType { PERM, TEMP, HOME, PATH };
    Point(void);
    /// by default a point it set to be permanent
    Point(const QString name, const double x, const double y, const PointType type = PERM);
    Point(const QString name, const Position position, const PointType type = PERM);

    /// Getters
    Position getPosition(void) const { return position; }
    QString getName(void) const { return name; }
    bool isPermanent(void) const { return (type == PERM); }
    bool isTemporary(void) const { return (type == TEMP); }
    bool isHome(void) const { return (type == HOME); }
    bool isPath(void) const { return (type == PATH); }
    PointType getType(void) const { return type; }
    QString getRobotName(void) const { return robotName; }

    /// Setters
    void setPosition(const double x, const double y) { position.setX(x); position.setY(y); }
    void setPosition(const Position _position) { position = _position; }
    void setName(const QString _name) { name = _name; }
    void setType(const PointType _type) { type = _type; }
    bool setHome(const PointType _type);
    void setRobotName(const QString name) { robotName = name; }


    /// a helper function to overload the << operator
    void display(std::ostream& stream) const;

    /**
     * @brief operator ==
     * @param point1
     * @return
     * To compare 2 points on their name + position
     */
    bool operator==(const Point& point1) const;

    /// compares the position of the current point with another position
    bool comparePos(const double x, const double y) const;
    bool comparePos(const Position pos) const;


private:
    QString name;
    Position position;
    PointType type;
    QString robotName;
};

/**
 * @brief operator <<
 * @param out
 * @param point
 * @return QDataStream&
 * Overloads the << and >> operators in order to be able to serialize a Point objet
 */
QDataStream& operator<<(QDataStream& out, const Point& point);
QDataStream& operator>>(QDataStream& in, Point& point);

std::ostream& operator <<(std::ostream& stream, const Point& point);


#endif // POINT_H
