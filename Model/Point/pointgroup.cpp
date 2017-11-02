#include "pointgroup.h"
#include "Model/Point/point.h"

PointGroup::PointGroup(QObject *parent) : QObject(parent), pointVector(QVector<QPointer<Point>>()) {}

void PointGroup::addPoint(const QString name, const double x, const double y, const bool displayed, const bool home, const int orientation){
    pointVector.push_back(QPointer<Point>(new Point(name, x, y, displayed, home, orientation, this)));
}

void PointGroup::addPoint(const QPointer<Point> point){
    pointVector.push_back(point);
}

void PointGroup::deletePoint(const QString name){
    for(int i = 0; i < pointVector.size(); i++)
        if(pointVector.at(i)->getName().compare(name) == 0)
            pointVector.remove(i);
}

void PointGroup::hideShow(const QString name){
    for(int i = 0; i < pointVector.size(); i++)
        if(pointVector.at(i)->getName().compare(name) == 0)
            pointVector.at(i)->setVisible(!pointVector.at(i)->isVisible());
}

QPointer<Point> PointGroup::takePoint(const QString name){
    for(int i = 0; i < pointVector.size(); i++)
        if(pointVector.at(i)->getName().compare(name) == 0)
            return pointVector.takeAt(i);
    /// not supposed to get here atm as this function is only called to move a point from a group to another
    Q_UNREACHABLE();
    return Q_NULLPTR;
}
