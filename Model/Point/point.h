#ifndef POINT_H
#define POINT_H

#include <QObject>

/**
 * @brief The Point class
 * This class provides a model for a point
 * A point is identified by a name that is unique, a Position objet whose class is described below and a boolean attribute
 * to distinguish permanent and temporary points
 */

class Point : public QObject{
    Q_OBJECT
public:
    Point(const QString _name, const QString groupName, const double _x, const double _y, const bool _visible, QObject *parent);
    QString getName(void) const { return name; }
    QString getGroupName(void) const { return groupName; }
    bool isVisible(void) const { return visible; }
    double getX(void) const { return x; }
    double getY(void) const { return y; }

private:
    QString name;
    bool visible;
    QString groupName;
    double x;
    double y;
};

#endif /// POINT_H
