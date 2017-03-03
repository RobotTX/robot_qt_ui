#include "mapcontroller.h"
#include <QDebug>
#include "Model/Map/map.h"

MapController::MapController(QObject *applicationWindow, QObject *parent) : QObject(parent){
    map = new Map(this);

    QObject *mapViewFrame = applicationWindow->findChild<QObject*>("mapViewFrame");
    if (mapViewFrame){
        connect(map, SIGNAL(setMap(QVariant)), mapViewFrame, SLOT(setMap(QVariant)));
        connect(map, SIGNAL(setMapState(QVariant,QVariant,QVariant)), mapViewFrame, SLOT(setMapState(QVariant,QVariant,QVariant)));
    } else {
        qDebug() << "MapController::MapController could not find the mapViewFrame";
        Q_UNREACHABLE();
    }

    map->initializeMap();
}
