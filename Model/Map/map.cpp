#include "map.h"
#include <assert.h>
#include <fstream>
#include <QDebug>
#include <QDir>
#include <QString>


Map::Map(QObject *parent) : QObject(parent) {
}

void Map::initializeMap(void){
    QString currentPathFile = QDir::currentPath() + QDir::separator() + "currentMap.txt";
    std::ifstream file(currentPathFile.toStdString(), std::ios::in);

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


        if(QFile(mapFile).exists()){
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

                mapImage = QImage(mapFile);
                origin = QPointF(originX, originY);
                dateTime = QDateTime::fromString(QString::fromStdString(_dateTime), "yyyy-MM-dd-hh-mm-ss");
                mapId = QUuid(QString::fromStdString(_mapId));
                pathFile.close();
                emit setMap(QVariant::fromValue(mapFile));
                emit setMapState(QVariant::fromValue(centerX), QVariant::fromValue(centerY), QVariant::fromValue(zoom));
            } else
                qDebug() << "Map::initializeMap could not find the map config file at :" << configPath;
        } else
            qDebug() << "Map::initializeMap could not find the map file at :" << mapFile;
    } else
        qDebug() << "Map::initializeMap could not find the currentMap file at :" << currentPathFile;
}

void Map::saveStateSlot(double posX, double posY, double zoom, QString mapSrc){
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
}

void Map::loadStateSlot(){
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
