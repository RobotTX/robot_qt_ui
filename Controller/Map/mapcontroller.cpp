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
        connect(this, SIGNAL(setMap(QVariant)), mapViewFrame, SLOT(setMap(QVariant)));
        connect(this, SIGNAL(setMapState(QVariant, QVariant, QVariant)), mapViewFrame, SLOT(setMapState(QVariant ,QVariant, QVariant)));
        connect(mapViewFrame, SIGNAL(saveState(double, double, double, QString)), this, SLOT(saveStateSlot(double, double, double, QString)));
        connect(mapViewFrame, SIGNAL(loadState()), this, SLOT(loadStateSlot()));
    } else {
        qDebug() << "MapController::MapController could not find the mapViewFrame";
        Q_UNREACHABLE();
    }

    QObject *mapMenuFrame = applicationWindow->findChild<QObject*>("mapMenuFrame");
    if (mapMenuFrame){
        connect(mapMenuFrame, SIGNAL(loadState()), this, SLOT(loadStateSlot()));

    } else {
        qDebug() << "MapController::MapController could not find the mapMenuFrame";
        Q_UNREACHABLE();
    }

    initializeMap();
}

void MapController::initializeMap(void){
    QString currentPathFile = QDir::currentPath() + QDir::separator() + "currentMap.txt";
    qDebug() << currentPathFile;
    std::ifstream file(currentPathFile.toStdString(), std::ios::in);

    if(file){
        /// We get the path of the map to use so that we can deduce the path of its config file
        std::string _dateTime, osef, stdMapFile;
        double centerX, centerY, zoom;
        file >> stdMapFile >> osef >> osef >> centerX >> centerY >> zoom >> osef >> osef >> osef >> _dateTime >> osef;
        file.close();

        /// our map file as a QString
        QString qMapFile = QString::fromStdString(stdMapFile);

        map->setMapFile(qMapFile);
        qDebug() << "Map::initializeMap full map path :" << qMapFile;
        /// We get the config file from the map file
        QString fileName = qMapFile;
        fileName.remove(0, fileName.lastIndexOf(QDir::separator()) + 1);
        fileName.remove(fileName.length() - 4, 4);
        QString configPath = QDir::currentPath() + QDir::separator() + "mapConfigs" + QDir::separator() + fileName + ".config";


        if(QFile(qMapFile).exists()){
            qDebug() << "Map::initializeMap config path :" << configPath;
            /// We get the map informations from the map config file
            std::ifstream pathFile(configPath.toStdString(), std::ios::in);
            if(pathFile){
                double originX, originY, resolution;
                std::string _mapId, mapFile;
                int height, width;
                pathFile >> osef >> height >> width >> osef >> osef >> osef >> originX >> originY >> resolution >> _mapId;
                qDebug() << "Map::initializeMap all info :" << QString::fromStdString(mapFile) << height << width
                         << centerX << centerY << originX << originY << resolution
                         << QString::fromStdString(_dateTime) << QString::fromStdString(_mapId);
                map->setHeight(height);
                map->setWidth(width);
                map->setMapImage(QImage(QString::fromStdString(mapFile)));
                map->setResolution(resolution);
                map->setOrigin(QPointF(originX, originY));
                map->setDateTime(QDateTime::fromString(QString::fromStdString(_dateTime), "yyyy-MM-dd-hh-mm-ss"));
                map->setId(QUuid(QString::fromStdString(_mapId)));
                pathFile.close();
                emit setMap(QVariant::fromValue(qMapFile));
                emit setMapState(QVariant::fromValue(centerX), QVariant::fromValue(centerY), QVariant::fromValue(zoom));
            } else
                qDebug() << "Map::initializeMap could not find the map config file at :" << configPath;
        } else
            qDebug() << "Map::initializeMap could not find the map file at :" << qMapFile;
    } else
        qDebug() << "Map::initializeMap could not find the currentMap file at :" << currentPathFile;
}

void MapController::saveStateSlot(double posX, double posY, double zoom, QString mapSrc){
    QString currentPathFile = QDir::currentPath() + QDir::separator() + "currentMap.txt";
    std::ofstream file(currentPathFile.toStdString(), std::ios::out | std::ios::trunc);

    /// saves the current configuration into the current configuration file
    if(file){
        /// the file comes as file:/urlOfMyFile.pgm so we need to remove the 6 first char
        mapSrc.remove(0,6);
        qDebug() << "Map::saveStateSlot called with following parameters";
        qDebug() << "map file - height - width - centerX - centerY - zoom - originX - originY - resolution - date - id";
        qDebug() << mapSrc << map->getHeight() << map->getWidth() << posX << posY
                 << zoom << map->getOrigin().x() << map->getOrigin().y() << map->getResolution()
                 << map->getDateTime().toString("yyyy-MM-dd-hh-mm-ss")
                 << map->getMapId().toString();

        file << mapSrc.toStdString() << " " << std::endl
             << map->getHeight() << " " << map->getWidth() << std::endl
             << posX << " " << posY << std::endl
             << zoom << std::endl
             << map->getOrigin().x() << " " << map->getOrigin().y() << std::endl
             << map->getResolution() << std::endl
             << map->getDateTime().toString("yyyy-MM-dd-hh-mm-ss").toStdString() << std::endl
             << map->getMapId().toString().toStdString();
        file.close();
    } else
        qDebug() << "Map::saveStateSlot could not find the currentMap file at :" << currentPathFile;
}

void MapController::loadStateSlot(){
    qDebug() << "Map::loadStateSlot called";
    QString currentPathFile = QDir::currentPath() + QDir::separator() + "currentMap.txt";
    std::ifstream file(currentPathFile.toStdString(), std::ios::in);

    if(file){
        /// We get the path of the map to use so that we can deduce the path of its config file
        std::string osef;
        double centerX, centerY, zoom;
        file >> osef >> osef >> osef >> centerX >> centerY >> zoom;
        file.close();
        emit setMapState(QVariant::fromValue(centerX), QVariant::fromValue(centerY), QVariant::fromValue(zoom));
    } else
        qDebug() << "Map::loadStateSlot could not find the currentMap file at :" << currentPathFile;
}

bool MapController::saveMapConfig(const std::string fileName, const double centerX, const double centerY, const double zoom) const {
    qDebug() << "MainWindow::saveMapConfig saving map to " << QString::fromStdString(fileName);
    std::ofstream file(fileName, std::ios::out | std::ios::trunc);
    if(file){
        file << map->getMapFile().toStdString() << " " << std::endl <<
                map->getHeight() << " " << map->getWidth() << std::endl
             << centerX << " " << centerY << std::endl
             << zoom << std::endl
             << map->getOrigin().x() << " " << map->getOrigin().y() << std::endl
             << map->getResolution() << std::endl
             << map->getMapId().toString().toStdString();
        file.close();
        return true;
    } else
        return false;
}
