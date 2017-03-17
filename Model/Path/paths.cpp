#include "paths.h"
#include <QMap>
#include <QDebug>
#include "Helper/helper.h"
#include "Model/Path/pathgroup.h"
#include "Model/Path/path.h"
#include "Model/Path/pathpoint.h"
#include "Model/Point/point.h"

Paths::Paths(QObject* parent) : QObject(parent), groups(QMap<QString, QPointer<PathGroup>>()) {}

void Paths::addGroup(const QString groupName){
    groups.insert(groupName, QPointer<PathGroup>(new PathGroup(this)));
}

void Paths::deleteGroup(const QString groupName){
    groups.remove(groupName);
}

void Paths::addPath(const QString groupName, const QString name){
    groups.value(groupName)->addPath(name);
}

void Paths::deletePath(const QString groupName, const QString name){
    groups.value(groupName)->deletePath(name);
}

void Paths::addPathPoint(const QString groupName, const QString pathName, const QString name, const double x, const double y, const int waitTime){
    groups.value(groupName)->addPathPoint(pathName, name, x, y, waitTime);
}

void Paths::deletePathPoint(const QString groupName, const QString pathName, const QString name){
    groups.value(groupName)->deletePathPoint(pathName, name);
}

void Paths::renameGroup(const QString newName, const QString oldName){
    groups.insert(newName, groups.take(oldName));
}

void Paths::movePath(const QString name, const QString oldGroup, const QString newGroup){
    qDebug() << "Paths::movePath";
}

bool Paths::checkGroupName(const QString name){
    return groups.find(Helper::formatName(name)) != groups.end();
}

void Paths::clearGoups(void){
    groups.clear();
}

void Paths::display(void){
    qDebug() << "\nPaths :";
    QMapIterator<QString, QPointer<PathGroup>> i(groups);
    while (i.hasNext()) {
        i.next();
        qDebug() << "\nGroup :" << i.key();

        QMapIterator<QString, QPointer<Path>> j(i.value()->getPaths());
        while (j.hasNext()) {
            j.next();
            qDebug() << "\nPath :" << j.key();
            for(int k = 0; k < j.value()->getPathPointVector().size(); k++){
                qDebug() << j.value()->getPathPointVector().at(k)->getPoint()->getName()
                         << j.value()->getPathPointVector().at(k)->getPoint()->getX()
                         << j.value()->getPathPointVector().at(k)->getPoint()->getY();
            }
        }
    }
}

