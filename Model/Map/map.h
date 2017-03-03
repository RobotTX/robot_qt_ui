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
    void initializeMap(void);

private slots:
    void saveStateSlot(double posX, double posY, double zoom, QString mapSrc);
    void loadStateSlot();

signals:
    void setMap(QVariant mapSrc);
    void setMapState(QVariant posX, QVariant posY, QVariant zoom);

private:

    /**
     * @brief resolution
     * The resolution of the map used to calculate the position of the robot
     * (the resolution of our map being different from the resolution of the map sent
     * affecting the robot's position)
     */
    float resolution;
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
};

#endif /// MAP_H
