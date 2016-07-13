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
            if(mapImage.pixelColor(j, i).red() == 254 && mapImage.pixelColor(j, i).green() == 254 && mapImage.pixelColor(j, i).blue() == 254){
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
    mapImage = QImage(width, height, QImage::Format_Grayscale8);
    int index = 0;
    /// We set each pixel of the image, the data received being
    /// a percent (0 to 100) of chance for a wall to be there
    /// -1 can also be received, meaning we do'ont know if there is a wall
    for(int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){
            int color = mapArrays.at(index);
            ///The percent is transform to a color (0 to 255),
            /// 205 being the grey
            if(mapArrays.at(index) < 0)
                color = 205;
            else if(mapArrays.at(index) < 30)
                color = 255;
            else if(mapArrays.at(index) < 70)
                color = 205;
            else {
                color = 0;

            }
            mapImage.setPixelColor(QPoint(j, height-1-i), QColor(color, color, color));

            index++;
        }
    }
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
