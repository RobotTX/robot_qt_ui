#ifndef POINT_H
#define POINT_H

#include <QObject>
#include <QPointF>

/**
 * @brief The Point class
 * This class provides a model for a point
 * A point is identified by a name that is unique, a Position objet whose class is described below and a boolean attribute
 * to distinguish permanent and temporary points
 */

class Point : public QObject {

public:
    Point(const QString _name, const double _x, const double _y, const bool _visible, QObject *parent);

    /// Getters
    QString getName(void) const { return name; }
    bool isVisible(void) const { return visible; }
    QPointF getPos(void) const { return pos; }

    /// Setters
    void setName(const QString _name) { name = _name; }
    void setVisible(const bool _visible) { visible = _visible; }
    void setX(const double _x) { pos.setX(_x); }
    void setY(const double _y) { pos.setY(_y); }

private:
    QString name;
    QPointF pos;
    bool visible;
};

#endif /// POINT_H
