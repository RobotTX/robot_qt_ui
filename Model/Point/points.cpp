#include "points.h"
#include <QDataStream>
#include <QDebug>
#include <QString>
#include "Model/Point/point.h"
#include "Helper/helper.h"

Points::Points(QObject* parent) : QObject(parent), groups(new QMap<QString, QVector<Point*>*>()) {

}

void Points::addGroup(QString groupName){
    groupName = Helper::formatName(groupName);
    if(!groups->contains(groupName)){
        //qDebug() << "Points::addGroup" << groupName;
        groups->insert(groupName, new QVector<Point*>());

        emit addGroupQml(QVariant::fromValue(indexOfGroup(groupName)),
                         QVariant::fromValue(groupName));
    }
}

void Points::addPoint(QString name, QString groupName, double x, double y, bool displayed){
    //qDebug() << "Points::addPoint" << groupName << name << x << y << displayed;
    addGroup(groupName);

    name = Helper::formatName(name);
    QVector<Point*>* group = groups->value(groupName);
    group->push_back(new Point(name, groupName, x, y, displayed, this));
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

void Points::deletePoint(QString name, QString groupName){
    /// we want ot delete a point so we remove it from the qml side
    emit removePointQml(QVariant::fromValue(indexOfPoint(name, groupName)));
    /// and we remove it from the c++ side
    QVector<Point*>* group = groups->value(groupName);
    for(int i = 0; i < group->size(); i++)
        if(group->at(i)->getName().compare(name) == 0)
            group->remove(i);
}

void Points::deleteGroup(QString name){
    /// if we want to delete a group, we delete all its points from the qml side and the c++ side
    if(groups->find(name) != groups->end()){
        emit removeGroupQml(QVariant::fromValue(indexOfGroup(name)),
                            QVariant::fromValue(indexOfPoint(groups->value(name)->last()->getName(), name)));
        groups->remove(name);
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
            if(group->at(j)->getName().compare(Helper::formatName(name)) == 0)
                return true;
        }
    }
    return false;
}

bool Points::checkGroupName(const QString name){
    return groups->find(Helper::formatName(name)) != groups->end();
}

void Points::renameGroup(QString newName, QString oldName){
    newName = Helper::formatName(newName);
    qDebug() << "Points::renameGroup from" << oldName << "to" << newName;
    groups->insert(newName, groups->take(oldName));
    emit renameGroupQml(newName, oldName);
}

void Points::moveTo(QString name, QString oldGroup, QString newGroup){
    qDebug() << "Points::move" << name << "from" << oldGroup << "to" << newGroup;
    /// TODO move to + on rename group
}
