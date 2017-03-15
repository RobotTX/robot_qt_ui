#include "points.h"
#include <QDataStream>
#include <QDebug>
#include <QString>
#include "Model/Point/point.h"

Points::Points(QObject* parent) : QObject(parent), groups(QMap<QString, QVector<QPointer<Point>>>()) {}

void Points::resetGroups(){

}

void Points::deletePointFromGroup(const QString group, const QString point_name){
    /// we remove the point from the c++ side
    for(int i = 0; i < groups[group].size(); i++)
        if(groups[group].at(i)->getName().compare(point_name) == 0)
            groups[group].remove(i);
}
