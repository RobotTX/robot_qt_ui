#include "Model/position.h"

namespace Helper {
    namespace Convert {
        Position pixelCoordToRobotCoord(const Position positionInPixels, double originX, double originY, double resolution, int height);
        Position robotCoordToPixelCoord(const Position positionInRobotCoordinates, double originX, double originY, double resolution, int height);
    }
}
