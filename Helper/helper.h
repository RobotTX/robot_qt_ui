#include "Model/Other/position.h"

/**
 * In this file we put all sorts of useful functions used by any classes
 */

namespace Helper {

    namespace Convert {

        Position pixelCoordToRobotCoord(const Position positionInPixels, double originX, double originY, double resolution, int height);
        Position robotCoordToPixelCoord(const Position positionInRobotCoordinates, double originX, double originY, double resolution, int height);
    }
}
