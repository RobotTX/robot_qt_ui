#include "map.h"
#include <QDebug>
#include <assert.h>

/// attributes are set to 0 because they will be updated later
Map::Map(): resolution(0), width(0), height(0), origin(Position()), mapId(QUuid()), modified(false) {}

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

QImage Map::getImageFromArray(const QByteArray& mapArrays, const bool fromPgm){

    qDebug() << "Map::getImageFromArray" << width << height << fromPgm;
    QImage image = QImage(width, height, QImage::Format_Grayscale8);

    uint32_t index = 0;

    /// depending on where we get the map from, the system of coordinates is not the same
    /// so the formula is adjusted using <shift> and <sign>
    int shift = 0;
    int sign = 1;
    if(!fromPgm){
        shift = height-1;
        sign = -1;
    }

    QVector<int> countVector;
    int countSum = 0;

    /// We set each pixel of the image
    for(int i = 0; i < mapArrays.size(); i+=5){
        int color = static_cast<int> (static_cast<uint8_t> (mapArrays.at(i)));

        uint32_t count = static_cast<uint32_t> (static_cast<uint8_t> (mapArrays.at(i+1)) << 24) + static_cast<uint32_t> (static_cast<uint8_t> (mapArrays.at(i+2)) << 16)
                        + static_cast<uint32_t> (static_cast<uint8_t> (mapArrays.at(i+3)) << 8) + static_cast<uint32_t> (static_cast<uint8_t> (mapArrays.at(i+4)));

        countVector.push_back(count);
        countSum += count;

        for(int j = 0; j < static_cast<int> (count); j++){
            if(index > width*width){
                qDebug() << "Map::getImageFromArray GOT A PROBLEM TO CHECK" << countVector << countSum << count;
                Q_UNREACHABLE();
                return image;
            }
            image.setPixelColor(QPoint(static_cast<int>(index%width), shift + sign * (static_cast<int>(index/width))), QColor(color, color, color));
            index++;
        }
    }

    return image;
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
