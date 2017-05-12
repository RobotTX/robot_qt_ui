#include "point.h"
#include <QDebug>
#include <QDataStream>
#include <iostream>


Point::Point(const QString _name, const double _x, const double _y, const bool _visible, const bool _home, const int _orientation, QObject *parent) :
    QObject(parent), name(_name), pos(QPointF(_x, _y)), visible(_visible), home(_home), orientation(_orientation)
{}
