#include "point.h"
#include <QDebug>
#include <QDataStream>
#include <iostream>

Point::Point(const QString _name, const double _x, const double _y, const bool _visible, QObject *parent = Q_NULLPTR) :
    QObject(parent), name(_name), pos(QPointF(_x, _y)), visible(_visible)
{}
