#include "map.h"
#include <assert.h>
#include <fstream>
#include <QDebug>
#include <QDir>
#include <QString>


Map::Map(QObject *parent) : QObject(parent) {
}

void Map::initializeMap(void){
    std::ifstream file((QDir::currentPath() + QDir::separator() + "currentMap.txt").toStdString(), std::ios::in);

    if(file){
        /// We get the path of the map to use so that we can deduce the path of its config file
        std::string _dateTime, osef, stdMapFile;
        double centerX, centerY, zoom;
        file >> stdMapFile >> osef >> osef >> centerX >> centerY >> zoom >> osef >> osef >> osef >> _dateTime >> osef;
        file.close();

        mapFile = QString::fromStdString(stdMapFile);
        qDebug() << "Map::initializeMap full map path :" << mapFile;
        /// We get the config file from the map file
        QString fileName = mapFile;
        fileName.remove(0, fileName.lastIndexOf(QDir::separator()) + 1);
        fileName.remove(fileName.length() - 4, 4);
        QString configPath = QDir::currentPath() + QDir::separator() + "mapConfigs" + QDir::separator() + fileName + ".config";

        qDebug() << "Map::initializeMap config path :" << configPath;

        /// We get the map informations from the map config file
        std::ifstream pathFile(configPath.toStdString(), std::ios::in);
        if(pathFile){
            double originX, originY;
            std::string _mapId;

            pathFile >> osef >> height >> width >> osef >> osef >> osef >> originX >> originY >> resolution >> _mapId;
            qDebug() << "Map::initializeMap all info :" << mapFile << height << width
                     << centerX << centerY << originX << originY << resolution
                     << QString::fromStdString(_dateTime) << QString::fromStdString(_mapId);

            origin = QPointF(originX, originY);
            dateTime = QDateTime::fromString(QString::fromStdString(_dateTime), "yyyy-MM-dd-hh-mm-ss");
            mapId = QUuid(QString::fromStdString(_mapId));
            pathFile.close();
            emit setMap(QVariant::fromValue(mapFile));
            emit setMapState(QVariant::fromValue(centerX), QVariant::fromValue(centerY), QVariant::fromValue(zoom));
        }
    }
}
