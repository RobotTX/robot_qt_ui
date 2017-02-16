#include "Model/Other/position.h"
#include <QString>

/**
 * In this file we put all sorts of useful functions used by any classes
 */

#define TESTING false

namespace Helper {

    namespace Convert {
        Position pixelCoordToRobotCoord(const Position positionInPixels, double originX, double originY, double resolution, int height);
        Position robotCoordToPixelCoord(const Position positionInRobotCoordinates, double originX, double originY, double resolution, int height);
    }

    /// to remove extra spaces like "A          word  " becomes "A word"
    QString formatName(const QString name);
}
