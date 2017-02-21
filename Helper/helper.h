#include "Model/Other/position.h"
#include <QString>
#include <QPair>

/**
 * In this file we put all sorts of useful functions used by any classes
 */

#define TESTING true

namespace Helper {

    namespace Convert {
        Position pixelCoordToRobotCoord(const Position positionInPixels, double originX, double originY, double resolution, int height);
        Position robotCoordToPixelCoord(const Position positionInRobotCoordinates, double originX, double originY, double resolution, int height);
    }

    namespace Date {

        /// returns true if the first date is later to the second date
        bool isLater(const QStringList& date, const QStringList& otherDate);
    }

    namespace File {

       QPair<QPair<QString, QString>, QStringList> getPathFromFile(const QString robotName);

       void updateHomeFile(const QString robotName, const Position& robot_home_position, const QStringList date);

       QPair<Position, QStringList> getHomeFromFile(const QString robotName);

    }

    namespace Prompt {

        int openConfirmMessage(const QString text);
    }

    /// to remove extra spaces like "A          word  " becomes "A word"
    QString formatName(const QString name);

    int mod(const int a, const int b);
}
