#include "helper.h"
#include <QTime>
#include <QCoreApplication>
#include <QDebug>

namespace Helper {

    namespace Convert {

        Position pixelCoordToRobotCoord(const Position positionInPixels, double originX, double originY, double resolution, int height){
            float xInRobotCoordinates = (positionInPixels.getX()) * resolution + originX;
            float yInRobotCoordinates = (-positionInPixels.getY() + height) * resolution + originY;
            return Position(xInRobotCoordinates, yInRobotCoordinates);
        }

        Position robotCoordToPixelCoord(const Position positionInRobotCoordinates, double originX, double originY, double resolution, int height){
            float xInPixelCoordinates = (-originX+ positionInRobotCoordinates.getX())/resolution;
            float yInPixelCoordinates = height - (-originY + positionInRobotCoordinates.getY()) / resolution;
            return Position(xInPixelCoordinates, yInPixelCoordinates);
        }
    }

    QString formatName(const QString name) {
        qDebug() << "GroupsPathsWidget::formatName called" << name;

        QString ret("");
        QStringList nameStrList = name.split(" ", QString::SkipEmptyParts);
        for(int i = 0; i < nameStrList.size(); i++){
            if(i > 0)
                ret += " ";
            ret += nameStrList.at(i);
        }
        if(name.size() > 0 && name.at(name.size()-1) == ' ')
            ret += " ";
        return ret;
    }
}
