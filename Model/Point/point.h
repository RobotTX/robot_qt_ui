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
    Point(const QString _name, const double x, const double y, const bool displayed, QObject *parent);
    QString getName(void) const { return name; }

private:
    QString name;
};

#endif /// POINT_H
