#include "points.h"
#include <QDataStream>
#include <QDebug>
#include "Model/Point/point.h"

Points::Points(QObject* parent) : QObject(parent), groups(new QMap<QString, QVector<Point*>*>()) {

}

void Points::addGroup(QString groupName){
    qDebug() << "Points::addGroup" << groupName;
    if(!groups->contains(groupName))
        groups->insert(groupName, new QVector<Point*>());
}

void Points::addPoint(QString groupName, QString name, double x, double y, bool displayed){
    qDebug() << "Points::addPoint" << groupName << name << x << y << displayed;
    if(!groups->contains(groupName))
        groups->insert(groupName, new QVector<Point*>());

    QVector<Point*>* group = groups->value(groupName);
    group->push_back(new Point(name, x, y, displayed, this));
}
