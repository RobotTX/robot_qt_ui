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

        /// We do not want to add the group "No Group" in the list but just display its points
        if(groupName.compare(NO_GROUP_NAME) != 0)
            /// We -1 the index of the group as we don't count the group "No Group"
            emit addGroupQml(QVariant::fromValue(indexOfGroup(groupName)-1),
                             QVariant::fromValue(groupName));
    }
}

void Points::addPoint(QString groupName, QString name, double x, double y, bool displayed){
    //qDebug() << "Points::addPoint" << groupName << name << x << y << displayed;
    addGroup(groupName);

    QVector<Point*>* group = groups->value(groupName);
    group->push_back(new Point(name, x, y, displayed, this));
    /// We -1 the index of the poit as we don't count the group "No Group"
    emit addPointQml(QVariant::fromValue(indexOfPoint(name, groupName)-1),
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
    if(groupName.isEmpty()){
        if(groups->find(name) != groups->end()){
            emit removeGroupQml(QVariant::fromValue(indexOfGroup(name) - 1),
                                QVariant::fromValue(indexOfPoint(groups->value(name)->at(groups->value(name)->size() - 1)->getName(), name) - 1));
            groups->remove(name);
        }
    } else {
        QVector<Point*>* group = groups->value(groupName);
        for(int i = 0; i < group->size(); i++){
            if(group->at(i)->getName().compare(name) == 0){
                emit removePointQml(QVariant::fromValue(indexOfPoint(name, groupName) - 1));
                group->remove(i);
            }
        }
    }
}
