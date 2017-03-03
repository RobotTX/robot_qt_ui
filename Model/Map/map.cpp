#include "map.h"
#include <QDebug>
#include <assert.h>
#include <fstream>
#include <QDir>

Map::Map(QObject *parent) : QObject(parent) {
    std::ifstream file((QDir::currentPath() + QDir::separator() + "currentMap.txt").toStdString(), std::ios::in);

    if(file){
        /// We get the path of the map to use so that we can deduce the path of its config file
        std::string _dateTime, osef;
        file >> mapFile >> osef >> osef >> osef >> osef >> osef >> osef >> osef >> osef >> _dateTime >> osef;
        file.close();

        qDebug() << "Map::Map full map path :" << QString::fromStdString(mapFile);
        /// We get the config file from the map file
        QString fileName = QString::fromStdString(mapFile);
        fileName.remove(0, fileName.lastIndexOf(QDir::separator()) + 1);
        fileName.remove(fileName.length() - 4, 4);
        QString configPath = QDir::currentPath() + QDir::separator() + "mapConfigs" + QDir::separator() + fileName + ".config";

        qDebug() << "Map::Map config path :" << configPath;

        /// We get the map informations from the map config file
        std::ifstream pathFile(configPath.toStdString(), std::ios::in);
        if(pathFile){
            double centerX, centerY, originX, originY;
            std::string _mapId;
            pathFile >> mapFile >> height >> width >> centerX >> centerY >> mapState.second >> originX >> originY >> resolution >> _mapId;
            qDebug() << "Map::Map all info :" << QString::fromStdString(mapFile) << height << width
                     << centerX << centerY << originX << originY << resolution
                     << QString::fromStdString(_dateTime) << QString::fromStdString(_mapId);
            mapState.first.setX(centerX);
            mapState.first.setY(centerY);
            origin = Position(originX, originY);
            dateTime = QDateTime::fromString(QString::fromStdString(_dateTime), "yyyy-MM-dd-hh-mm-ss");
            mapId = QUuid(QString::fromStdString(_mapId));
            pathFile.close();
        }
    }

    if(!mapFile.empty()){
        setMapFromFile(QString::fromStdString(mapFile));
}
