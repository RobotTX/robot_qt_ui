#include "point.h"
#include <QDebug>
#include <QDataStream>
#include <iostream>

Point::Point(const QString _name, const double _x, const double _y, const bool _visible, QObject *parent) :
    QObject(parent), name(_name), visible(_visible), x(_x), y(_y) {}
