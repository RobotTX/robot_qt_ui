#include "mapcontroller.h"
#include <QDebug>
#include <QDir>
#include <QFileDialog>
#include <fstream>
#include <QAbstractListModel>

MapController::MapController(QObject *applicationWindow, QObject *parent) : QObject(parent){
    map = new Map(this);

    QObject *mapViewFrame = applicationWindow->findChild<QObject*>("mapViewFrame");
    if (mapViewFrame){
        connect(map, SIGNAL(setMap(QVariant)), mapViewFrame, SLOT(setMap(QVariant)));
        connect(map, SIGNAL(setMapState(QVariant, QVariant, QVariant)), mapViewFrame, SLOT(setMapState(QVariant ,QVariant, QVariant)));
        connect(mapViewFrame, SIGNAL(saveState(double, double, double, QString)), map, SLOT(saveStateSlot(double, double, double, QString)));
        connect(mapViewFrame, SIGNAL(loadState()), map, SLOT(loadStateSlot()));
    } else {
        qDebug() << "MapController::MapController could not find the mapViewFrame";
        Q_UNREACHABLE();
    }

    QObject *mapMenuFrame = applicationWindow->findChild<QObject*>("mapMenuFrame");
    if (mapMenuFrame){
        connect(mapMenuFrame, SIGNAL(loadState()), map, SLOT(loadStateSlot()));

    } else {
        qDebug() << "MapController::MapController could not find the mapMenuFrame";
        Q_UNREACHABLE();
    }

    QObject* _appWindow = applicationWindow->findChild<QObject*>("applicationWindow");

    if(_appWindow == 0){
        qDebug() << "no app window found";
        connect(_appWindow, SIGNAL(mapConfig(QObject*, double, double, double)), this, SLOT(saveMapConfig(QObject*, double, double, double)));
    } else {
        qDebug() << "MapController::MapController could not find the applicationWindow";
        Q_UNREACHABLE();
    }

    map->initializeMap();
}

void MapController::saveMapConfig(double zoom, double centerX, double centerY) const {
    qDebug() << "MapController::saveMapConfig called with" << zoom << centerX << centerY;


/*
    std::string fileName((QDir::currentPath() + QDir::separator() + "mapConfigs" + QDir::separator() + getMapFile()).toStdString());

    qDebug() << "MainWindow::saveMapConfig saving map to " << QString::fromStdString(fileName);
    std::ofstream file(fileName, std::ios::out | std::ios::trunc);
    if(file){
        file << map->getMapFile().toStdString() << " " << std::endl <<
                map->getHeight() << " " << map->getWidth() << std::endl
             << map->getMapState().first.x() << " " << map->getMapState().first.y() << std::endl
             << map->getMapState().second << std::endl
             << map->getOrigin().getX() << " " << map->getOrigin().getY() << std::endl
             << map->getResolution() << std::endl
             << map->getMapId().toString().toStdString();
        file.close();
        return true;
    } else
        return false;
    QString currentPathFile = QDir::currentPath() + QDir::separator() + "currentMap.txt";
    std::ofstream file(currentPathFile.toStdString(), std::ios::out | std::ios::trunc);

    /// saves the current configuration into the current configuration file
    if(file){
        /// the file comes as file:/urlOfMyFile.pgm so we need to remove the 6 first char
        mapSrc.remove(0,6);
        qDebug() << "Map::saveStateSlot called with following parameters";
        qDebug() << "map file - height - width - centerX - centerY - zoom - originX - originY - resolution - date - id";
        qDebug() << mapSrc<< height << width << posX << posY
                 << zoom << origin.x() << origin.y() << resolution
                 << dateTime.toString("yyyy-MM-dd-hh-mm-ss")
                 << mapId.toString();

        file << mapSrc.toStdString() << " " << std::endl
             << height << " " << width << std::endl
             << posX << " " << posY << std::endl
             << zoom << std::endl
             << origin.x() << " " << origin.y() << std::endl
             << resolution << std::endl
             << dateTime.toString("yyyy-MM-dd-hh-mm-ss").toStdString() << std::endl
             << mapId.toString().toStdString();
        file.close();
    } else
        qDebug() << "Map::saveStateSlot could not find the currentMap file at :" << currentPathFile;

        */
}
