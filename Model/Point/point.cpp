#include "point.h"
#include <QDebug>
#include <QDataStream>
#include <iostream>

Point::Point(const QString _name, const double x, const double y, const bool displayed, QObject* parent) : QObject(parent), name(_name){

}
