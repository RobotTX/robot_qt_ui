#ifndef MAP_H
#define MAP_H

#include <QObject>
#include <QImage>
#include <QPointF>
#include <QDateTime>
#include <QUuid>
#include <QVariant>

/**
 * @brief The Map class
 * The model class for the Map,
 * contains the map as a QImage and its width, height, resolution and the origin
 */
class Map : public QObject {

    Q_OBJECT

public:
    Map(QObject* parent);

    QString getMapFile(void) const { return mapFile; }
    QImage getMapImage(void) const { return mapImage; }
    int getHeight(void) const { return height; }
    int getWidth(void) const { return width; }
    QPointF getOrigin(void) const { return origin; }
    double getResolution(void) const { return resolution; }
    QDateTime getDateTime(void) const { return dateTime; }
    QUuid getMapId(void) const { return mapId; }

    void setMapFile(const QString file) { mapFile = file; }
    void setId(const QUuid id) { mapId = id; }
    void setDateTime(const QDateTime date) { dateTime = date; }
    void setMapImage(const QImage image) { mapImage = image; }
    void setOrigin(const QPointF _origin) { origin = _origin; }
    void setHeight(const int _height) { height = _height; }
    void setWidth(const int _width) { width = _width; }
    void setResolution(const double _resolution) { resolution = _resolution; }

private:

    /**
     * @brief resolution
     * The resolution of the map used to calculate the position of the robot
     * (the resolution of our map being different from the resolution of the map sent
     * affecting the robot's position)
     */
    double resolution;
    int width;
    int height;

    /**
     * @brief origin
     * The origin of the map, the position of the robot depending of this origin
     * (which can be anywhere in the map) and not our map origin (top-left corner)
     */
    QPointF origin;

    /**
     * @brief dateTime
     * The last time the map was modified
     */
    QDateTime dateTime;

    /**
     * @brief mapId
     * The id of the map, used to make sure all robots linked to this application have the same map
     */
    QUuid mapId;

    /**
     * @brief mapFile
     * File where the currently used map is stored
     */
    QString mapFile;
    QImage mapImage;
};

#endif /// MAP_H
