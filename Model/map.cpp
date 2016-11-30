#include "map.h"
#include <QDebug>
#include <assert.h>


Map::Map(): resolution(0), width(0), height(0), origin(Position()), mapId(QUuid()) {}


void Map::setMapFromFile(const QString fileName){
    /// Qt has is own function to create a QImage from a PGM file
    mapImage = QImage(fileName,"PGM");
    width = mapImage.width();
    height = mapImage.height();
}

void Map::setMapFromArray(const QByteArray& mapArrays, bool fromPgm){
    qDebug() << "Map::setMapFromArray called" << mapArrays.size();
    mapImage = QImage(width, height, QImage::Format_Grayscale8);
    uint32_t index = 0;

    /// depending on where we get the map from the system of coordinates is not the same
    /// so the formula is adjusted using <shift> and <sign>
    int shift = 0;
    int sign = 1;
    if(!fromPgm){
        shift = height-1;
        sign = -1;
    }

    /// We set each pixel of the image, the data received being
    for(int i = 0; i < mapArrays.size(); i+=5){
        int color = static_cast<int> (static_cast<uint8_t> (mapArrays.at(i)));

        uint32_t count = static_cast<uint32_t> (static_cast<uint8_t> (mapArrays.at(i+1)) << 24) + static_cast<uint32_t> (static_cast<uint8_t> (mapArrays.at(i+2)) << 16)
                        + static_cast<uint32_t> (static_cast<uint8_t> (mapArrays.at(i+3)) << 8) + static_cast<uint32_t> (static_cast<uint8_t> (mapArrays.at(i+4)));

        for(int j = 0; j < (int) count; j++){
            mapImage.setPixelColor(QPoint(static_cast<int>(index%width), shift + sign * (static_cast<int>(index/width))), QColor(color, color, color));
            index++;
        }
    }
}

void Map::saveToFile(const QString fileName){
    /// Qt has is own function to save the QImage to a PGM file
    mapImage.save(fileName, "PGM");
}

void Map::setDateTime(const QDateTime _dateTime) {
    dateTime = (_dateTime.isValid()) ? _dateTime : QDateTime::currentDateTime();
}
