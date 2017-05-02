#include <QString>
#include <QPair>
#include <QPointF>
#include <QImage>

/**
 * In this file we put all sorts of useful functions used by any classes
 */

#define NO_GROUP_NAME "No Group"

#define PORT_ROBOT_UPDATE 6000
//#define PORT_MAP_METADATA 4000
#define PORT_ROBOT_POS 4001
#define PORT_MAP 4002
#define PORT_CMD 5600
#define PORT_NEW_MAP 5601
#define PORT_TELEOP 5602
#define PORT_LOCAL_MAP 5605
#define PORT_PARTICLE_CLOUD 4005

namespace Helper {

    enum SETTING_MAP_CHOICE { ALWAYS_ROBOT, ALWAYS_APPLICATION, ALWAYS_ASK , ALWAYS_NEW , ALWAYS_OLD };

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

    namespace Image {

        QPair<QImage, QPoint> crop(const QImage& image, const int n);
    }

    /// to remove extra spaces like "A          word  " becomes "A word"
    QString formatName(const QString name);

    int mod(const int a, const int b);

    QString getAppPath(void);
}
