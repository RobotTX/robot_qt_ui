#include <QString>
#include <QPair>
#include <QPointF>

/**
 * In this file we put all sorts of useful functions used by any classes
 */

#define TESTING false

namespace Helper {

    namespace Convert {
        QPointF pixelCoordToRobotCoord(const QPointF positionInPixels, double originX, double originY, double resolution, int height);
        QPointF robotCoordToPixelCoord(const QPointF positionInRobotCoordinates, double originX, double originY, double resolution, int height);
    }

    namespace Date {

        /// returns true if the first date is later to the second date
        bool isLater(const QStringList& date, const QStringList& otherDate);
    }

    namespace File {

       QPair<QPair<QString, QString>, QStringList> getPathFromFile(const QString robotName);

       void updateHomeFile(const QString robotName, const QPointF& robot_home_position, const QStringList date);

       QPair<QPointF, QStringList> getHomeFromFile(const QString robotName);

    }

    /// to remove extra spaces like "A          word  " becomes "A word"
    QString formatName(const QString name);

    int mod(const int a, const int b);
}
