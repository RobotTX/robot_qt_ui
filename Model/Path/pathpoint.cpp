#include "pathpoint.h"
#include "Model/Point/point.h"

PathPoint::PathPoint(const QString _name, const double _x, const double _y, const int _waitTime, QObject *parent)
    : QObject(parent), waitTime(_waitTime), point(new Point(_name, _x, _y, true, false, 0, this)) {}
