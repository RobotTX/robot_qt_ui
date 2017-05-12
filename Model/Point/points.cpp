#include "points.h"
#include <QDataStream>
#include <QDebug>
#include <QString>
#include "Helper/helper.h"
#include "Model/Point/point.h"
#include "Model/Point/pointgroup.h"

Points::Points(QObject* parent) : QObject(parent), groups(QMap<QString, QPointer<PointGroup>>()) {}

void Points::addGroup(const QString groupName){
    groups.insert(groupName, QPointer<PointGroup>(new PointGroup(this)));
}

void Points::addPoint(const QString groupName, const QString name, const double x, const double y, const bool displayed, const bool home, const int orientation){
    groups.value(groupName)->addPoint(name, x, y, displayed, home, orientation);
}

void Points::deletePoint(const QString groupName, const QString name){
    groups.value(groupName)->deletePoint(name);
}

void Points::deleteGroup(const QString groupName){
    groups.remove(groupName);
}

void Points::hideShow(const QString groupName, const QString name){
    groups.value(groupName)->hideShow(name);
}

void Points::renameGroup(const QString newName, const QString oldName){
    groups.insert(newName, groups.take(oldName));
}

void Points::movePoint(const QString name, const QString oldGroup, const QString newGroup){
    groups.value(newGroup)->addPoint(groups.value(oldGroup)->takePoint(name));
}

bool Points::checkGroupName(const QString name){
    return name.isEmpty() || groups.find(name) != groups.end();
}

void Points::clearGoups(){
    groups.clear();
}
