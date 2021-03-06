#include "path.h"
#include "Model/Path/pathpoint.h"
#include "Model/Point/point.h"

Path::Path(QObject *parent) : QObject(parent), pathPointVector(QVector<QPointer<PathPoint>>()) {}

void Path::addPathPoint(const QString name, const double x, const double y, const int waitTime, const int orientation, const QString speechName, const QString speechContent, const int speechTime){
    pathPointVector.push_back(QPointer<PathPoint>(new PathPoint(name, x, y, waitTime, orientation, speechName, speechContent, speechTime, this)));
}

void Path::deletePathPoint(const QString name){
    for(int i = 0; i < pathPointVector.size(); i++)
        if(pathPointVector.at(i)->getPoint()->getName().compare(name) == 0)
            pathPointVector.remove(i);
}
