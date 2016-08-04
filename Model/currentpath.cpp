#include "currentpath.h"
#include "Model/pathpoint.h"
#include "Model/points.h"
#include "View/mapview.h"

CurrentPath::CurrentPath(MainWindow* const &parent, MapView* const &mapView, std::shared_ptr<Points> _points) : QObject(parent), points(_points){
    path = std::shared_ptr<QVector<std::shared_ptr<PathPoint>>>(new QVector<std::shared_ptr<PathPoint>>());
}

void CurrentPath::reset(void){
    qDebug() << "CurrentPath::reset called";
    path->clear();
}
