#include "points.h"
#include <QDataStream>
#include <QDebug>
#include <QString>
#include "Model/Point/point.h"

Points::Points(QObject* parent) : QObject(parent), groups(new QMap<QString, QVector<Point*>*>()) {

}

void Points::addGroup(QString groupName){
    if(!groups->contains(groupName)){
        //qDebug() << "Points::addGroup" << groupName;
        groups->insert(groupName, new QVector<Point*>());

        /// We -1 the index of the group as we don't count the group "No Group"
        emit addGroupQml(QVariant::fromValue(indexOfGroup(groupName)),
                         QVariant::fromValue(groupName));
    }
}

void Points::addPoint(QString name, QString groupName, double x, double y, bool displayed){
    //qDebug() << "Points::addPoint" << groupName << name << x << y << displayed;
    addGroup(groupName);

    QVector<Point*>* group = groups->value(groupName);
    group->push_back(new Point(name, this));
    /// We -1 the index of the point as we don't count the group "No Group"
    emit addPointQml(QVariant::fromValue(indexOfPoint(name, groupName)),
                     QVariant::fromValue(name),
                     QVariant::fromValue(displayed),
                     QVariant::fromValue(groupName),
                     QVariant::fromValue(x),
                     QVariant::fromValue(y));
}

int Points::indexOfPoint(QString pointName, QString groupName){
    QMapIterator<QString, QVector<Point*>*> i(*groups);
    int index(0);
    while (i.hasNext()) {
        i.next();
        index++;

        QVector<Point*>* group = i.value();
        for(int j = 0; j < group->size(); j++){
            index++;
            if(i.key().compare(groupName) == 0 && group->at(j)->getName().compare(pointName) == 0)
                return index-1;
        }
    }

    return -1;
}

int Points::indexOfGroup(QString groupName){
    QMapIterator<QString, QVector<Point*>*> i(*groups);
    int index(0);
    while (i.hasNext()) {
        i.next();
        index++;
        if(i.key().compare(groupName) == 0)
            return index-1;
        QVector<Point*>* group = i.value();
        for(int j = 0; j < group->size(); j++)
            index++;
    }

    return -1;
}

void Points::deletePointOrGroup(QString name, QString groupName){
    //qDebug() << "Points::deletePointOrGroup" << name << groupName;
    /// if we want to delete a group, we delete all its points
    if(groupName.isEmpty()){
        if(groups->find(name) != groups->end()){
            emit removeGroupQml(QVariant::fromValue(indexOfGroup(name)),
                                QVariant::fromValue(indexOfPoint(groups->value(name)->at(groups->value(name)->size())->getName(), name) - 1));
            groups->remove(name);
        }
    } else {
        /// TODO check if work correctly since made a few modif
        /// we want ot delete a point so we remove it on the qml side
        emit removePointQml(QVariant::fromValue(indexOfPoint(name, groupName)));
        /// and we remove it on the c++ side
        QVector<Point*>* group = groups->value(groupName);
        for(int i = 0; i < group->size(); i++)
            if(group->at(i)->getName().compare(name) == 0)
                group->remove(i);
    }
}

void Points::hideShow(QString name, QString groupName, bool show){
    /// If it's a group
    if(groupName.isEmpty())
        emit hideShowQml(QVariant::fromValue(indexOfGroup(name)), QVariant::fromValue(!show));
    else
        emit hideShowQml(QVariant::fromValue(indexOfPoint(name, groupName)), QVariant::fromValue(!show));
}

bool Points::checkPointName(const QString name){
    QMapIterator<QString, QVector<Point*>*> i(*groups);
    while (i.hasNext()) {
        i.next();

        QVector<Point*>* group = i.value();
        for(int j = 0; j < group->size(); j++){
            if(group->at(j)->getName().compare(name) == 0)
                return true;
        }
    }
    return false;
}

bool Points::checkGroupName(const QString name){
    return groups->find(name) != groups->end();
}

