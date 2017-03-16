#include "pathpoint.h"
#include "Model/Point/point.h"

PathPoint::PathPoint(const QString _name, const double _x, const double _y, const int _waitTime, QObject *parent)
    : QObject(parent), point(new Point(_name, _x, _y, true, this)), waitTime(_waitTime) {

}
