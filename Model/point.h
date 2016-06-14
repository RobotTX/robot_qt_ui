#ifndef POINT_H
#define POINT_H

class QDataStream;
#include <QString>

/**
 * @brief The Point class
 * This class provides a model for a point
 * A point is identified by a name that is unique, a Position objet whose class is described below and a boolean attribute
 * to distinguish permanent and temporary points
 */

class Point
{
    /**
     * @brief The Position class
     * This class provides a model for a position, it is meant to be used by a point and therefore known to it only
     * A point is identified by two coordinates represented as floats numbers
     */
    class Position {
    public:
        Position(void): _x(0.0), _y(0.0) {}
        Position(const float x, const float y): _x(x), _y(y) {}

        float getX(void) const { return _x; }
        float getY(void) const { return _y; }

        void setX(const float x) { _x = x; }
        void setY(const float y) { _y = y; }

    private:
        float _x;
        float _y;
    };

public:
    Point(void);
    /// by default a point it set to be permanent
    Point(const QString name, const float x, const float y, const bool permanent = true);
    Point(const QString name, const Position position, const bool _displayed = false, const bool permanent = true);

    Position getPosition(void) const { return position; }

    QString getName(void) const { return name; }

    bool isDisplayed(void) const { return displayed; }
    void setDisplayed(const bool _displayed) { displayed = _displayed; }

    bool isPermanent(void) const { return permanent; }
    void setPermanent(const bool _permanent) { permanent = _permanent; }

    void setName(const QString _name) { name = _name; }
    void setPosition(const float x, const float y) { position.setX(x); position.setY(y); }

    /// a helper function to overload the << operator
    void display(std::ostream& stream) const;

    bool operator==(const Point& point1) const;

    bool comparePos(const float x, const float y) const;

private:
    QString name;
    Position position;
    /// a point can be displayed or not on the map
    bool displayed;
    bool permanent;

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
