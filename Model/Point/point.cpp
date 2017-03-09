#include "point.h"
#include <QDebug>
#include <QDataStream>
#include <iostream>

Point::Point(const QString _name, QObject* parent) : QObject(parent), name(_name){

}
