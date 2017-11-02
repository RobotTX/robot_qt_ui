#include "pathpoint.h"
#include "Model/Point/point.h"

PathPoint::PathPoint(const QString _name, const double _x, const double _y, const int _waitTime, const int _orientation, QObject *parent)
    : QObject(parent), waitTime(_waitTime), point(new Point(_name, _x, _y, true, false, _orientation, this))
{}

