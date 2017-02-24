#include "map.h"
#include <QDebug>
#include <assert.h>
#include <fstream>
#include <QDir>

/// attributes are set to 0 because they will be updated later
Map::Map(): modified(false) {

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

    if(!mapFile.compare("")){
        setMapFromFile(":/maps/map.pgm");
        mapState.second = 1;
        height = 608;
        width = 320;
        mapState.first.setX(0);
        mapState.first.setY(0);
        resolution = 0.05;
        origin = Position(0, 0);
        dateTime = QDateTime(QDate::fromString("1970-01-01", "yyyy-mm-dd"));
        mapId = QUuid();
    } else
        setMapFromFile(QString::fromStdString(mapFile));
}

void Map::setMapFromFile(const QString fileName){
    /// Qt has is own function to create a QImage from a PGM file
    mapImage = QImage(fileName,"PGM");
    width = mapImage.width();
    height = mapImage.height();
    /// Doesn't count as a modification to save as we load the map from a file
    modified = false;
}

void Map::setMapFromArray(const QByteArray& mapArrays, bool fromPgm){
    qDebug() << "Map::setMapFromArray called" << mapArrays.size();

    /// This map comes from a robot so we want to tell the user to save the new map when closing the app
    modified = true;

    mapImage = getImageFromArray(mapArrays, fromPgm);
}

QImage Map::getImageFromArray(const QByteArray& mapArrays, const int map_width, const int map_height, const bool fromPgm){

    qDebug() << "Map::getImageFromArray" << map_width << map_height << fromPgm;
    QImage image = QImage(map_width, map_height, QImage::Format_Grayscale8);

    uint32_t index = 0;

    /// depending on where we get the map from, the system of coordinates is not the same
    /// so the formula is adjusted using <shift> and <sign>
    int shift = 0;
    int sign = 1;
    if(!fromPgm){
        shift = map_height-1;
        sign = -1;
    }

    QVector<int> countVector;
    int countSum = 0;

    /// We set each pixel of the image
    for(int i = 0; i < mapArrays.size(); i += 5){
        int color = static_cast<int> (static_cast<uint8_t> (mapArrays.at(i)));

        uint32_t count = static_cast<uint32_t> (static_cast<uint8_t> (mapArrays.at(i+1)) << 24) + static_cast<uint32_t> (static_cast<uint8_t> (mapArrays.at(i+2)) << 16)
                        + static_cast<uint32_t> (static_cast<uint8_t> (mapArrays.at(i+3)) << 8) + static_cast<uint32_t> (static_cast<uint8_t> (mapArrays.at(i+4)));

        countVector.push_back(count);
        countSum += count;

        for(int j = 0; j < static_cast<int> (count); j++){
            /// Sometimes we receive too much informations so we need to check
            if(index >= static_cast<uint>(map_width*map_height))
                return image;

            image.setPixelColor(QPoint(static_cast<int>(index%map_width), shift + sign * (static_cast<int>(index/map_width))), QColor(color, color, color));
            index++;
        }
    }

    return image;
}

QImage Map::getImageFromArray(const QByteArray& mapArrays, const bool fromPgm){

    qDebug() << "Map::getImageFromArray" << width << height << fromPgm;
    return getImageFromArray(mapArrays, width, height, fromPgm);
}

void Map::saveToFile(const QString fileName){
    /// Qt has is own function to save the QImage to a PGM file
    mapImage.save(fileName, "PGM");

    /// When the map is saved, no need to tell the user to save it again when closing the app
    modified = false;
}

void Map::setDateTime(const QDateTime _dateTime) {
    /// if the time is not valid we use the current date
    dateTime = (_dateTime.isValid()) ? _dateTime : QDateTime::currentDateTime();
}
