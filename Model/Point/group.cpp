#include "group.h"
#include "Model/Point/point.h"

Group::Group(QObject *parent) : QObject(parent), pointVector(QVector<QPointer<Point>>()){

}

void Group::addPoint(const QString name, const double x, const double y, const bool displayed){
    pointVector.push_back(QPointer<Point>(new Point(name, x, y, displayed, this)));
}

void Group::deletePoint(const QString name){
    for(int i = 0; i < pointVector.size(); i++)
        if(pointVector.at(i)->getName().compare(name) == 0)
            pointVector.remove(i);
}

void Group::hideShow(const QString name){
    for(int i = 0; i < pointVector.size(); i++)
        if(pointVector.at(i)->getName().compare(name) == 0)
            pointVector.at(i)->setVisible(pointVector.at(i)->isVisible());
}

void Group::addPoint(const QPointer<Point> point){
    pointVector.push_back(point);
}

QPointer<Point> Group::takePoint(const QString name){
    for(int i = 0; i < pointVector.size(); i++)
        if(pointVector.at(i)->getName().compare(name) == 0)
            return pointVector.takeAt(i);

    Q_UNREACHABLE();
    return Q_NULLPTR;
}
