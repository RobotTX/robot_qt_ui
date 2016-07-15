#include "map.h"
#include <QDebug>

Map::Map(){
    resolution = 0;
    width = 0;
    height = 0;
    origin = Position();
}

void Map::setMapFromFile(const QString fileName){
    qDebug() << "Loaded image from file";
    /// Qt has is own function to create a QImage from a PGM file
    mapImage = QImage(fileName,"PGM");
    width = mapImage.width();
    height = mapImage.height();
    setRectangle();
}

void Map::setRectangle(void){
    /// to determine the rectangle of the map
    double upperBound(0.0);
    double lowerBound(height);
    double righterBound(0.0);
    double lefterBound(width);
    for(int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){
            if(mapImage.pixelColor(j, i).red() >= 254){
                if(i > upperBound)
                    upperBound = i;
                else if(i < lowerBound)
                    lowerBound = i;
                else if(j < lefterBound)
                    lefterBound = j;
                else if(j > righterBound)
                    righterBound = j;
            }
        }
    }
    rect = QRect(QPoint(lefterBound, lowerBound), QPoint(righterBound, upperBound));
    qDebug() << "just set map rectangle " << rect;
    center = QPointF(rect.topLeft().x() + rect.bottomRight().x() /2,
            (rect.topLeft().y() + rect.bottomRight().y()) /2);
}

// this code is temporarily dead

void Map::setMapFromArray(const QByteArray& mapArrays){
    qDebug() << "setMapFromArray called" << mapArrays.size();
    mapImage = QImage(width, height, QImage::Format_Grayscale8);
    uint32_t index = 0;
    int indexI = 0;
    int indexJ = 0;

    /// We set each pixel of the image, the data received being
    for(int i = 0; i < mapArrays.size(); i+=5){
        int color = (int) ((uint8_t) mapArrays.at(i));

        uint32_t count = (uint32_t) (((uint8_t)mapArrays.at(i+1)) << 24) + (uint32_t) (((uint8_t)mapArrays.at(i+2)) << 16)
                        + (uint32_t) (((uint8_t)mapArrays.at(i+3)) << 8) + (uint32_t) ((uint8_t) mapArrays.at(i+4));

        for(int j = 0; j < (int) count; j++){
            mapImage.setPixelColor(QPoint((int) (index%width), height-1-((int) (index/width))), QColor((int) color, color, color));
            index++;
        }
    }
    qDebug() << "Last indexes :" << index << indexJ << indexI;
}

void Map::saveToFile(const QString fileName){
    /// Qt has is own function to save the QImage to a PGM file
    bool status = mapImage.save(fileName, "PGM");
    if(status){
        qDebug() << "Map saved";
    } else {
        qDebug() << "Error : map not saved";
    }
}
