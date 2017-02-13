#include "helper.h"
#include <QTime>
#include <QCoreApplication>

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

    namespace Thread {

        void delay(const int ms){
            QTime dieTime = QTime::currentTime().addMSecs(ms);
            while (QTime::currentTime() < dieTime)
                QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
        }
    }
}
