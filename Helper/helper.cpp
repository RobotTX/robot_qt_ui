#include "helper.h"

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
}
