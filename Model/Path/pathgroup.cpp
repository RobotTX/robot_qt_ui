#include "pathgroup.h"
#include "Model/Path/path.h"

PathGroup::PathGroup(QObject* parent) : QObject(parent), paths(QMap<QString, QPointer<Path>>()){

}

void PathGroup::addPath(const QString name){
    paths.insert(name, QPointer<Path>(new Path(this)));
}

void PathGroup::addPath(const QString name, const QPointer<Path> path){
    paths.insert(name, path);
}

void PathGroup::deletePath(const QString name){
    paths.remove(name);
}

void PathGroup::addPathPoint(const QString pathName, const QString name, const double x, const double y, const int waitTime){
    paths.value(pathName)->addPathPoint(name, x, y, waitTime);
}

void PathGroup::deletePathPoint(const QString pathName, const QString name){
    paths.value(pathName)->deletePathPoint(name);
}

QPointer<Path> PathGroup::takePath(const QString name){
    return paths.take(name);
}
