#ifndef MAP_H
#define MAP_H

#include "Model/position.h"
#include <QImage>
#include <QObject>
#include <QDateTime>
#include <QUuid>

/**
 * @brief The Map class
 * The model class for the Map,
 * contains the map as a QImage and its width, height, resolution and the origin
 */
class Map : public QObject
{
    Q_OBJECT
public:
    explicit Map();

    QImage getMapImage(void) const { return mapImage; }
    float getResolution(void) const { return resolution; }
    int getWidth(void) const { return width; }
    int getHeight(void) const { return height; }
    Position getOrigin(void) const { return origin; }
    QRect getRect(void) const { return rect; }
    QPointF getCenter(void) const { return center; }
    QDateTime getDateTime(void) const { return dateTime; }
    QUuid getMapId(void) const { return mapId; }
    bool getModified(void) const { return modified; }

    void setResolution(const float _resolution) { resolution = _resolution; }
    void setWidth(const int _width) { width = _width; }
    void setHeight(const int _height) { height = _height; }
    void setOrigin(const Position _origin) { origin = _origin; }
    void setMapImage(const QImage _mapImage) { mapImage = _mapImage; modified = true; }
    void setDateTime(const QDateTime _dateTime);
    void setMapId(const QUuid _mapId) { mapId = _mapId; }
    void setModified(const bool _modified) { modified = _modified; }

    /**
     * @brief setMapFromArray
     * @param mapArrays
     * Create the QImage mapImage from an array of byte
     */
    void setMapFromArray(const QByteArray& mapArrays, bool fromPgm);

    /**
     * @brief setMapFromFile
     * @param fileName
     * Create/load the QImage mapImage from a PGM file
     */
    void setMapFromFile(const QString fileName);

    /**
     * @brief saveToFile
     * @param fileName
     * Save the current mapImage to a PGM file
     */
    void saveToFile(const QString fileName);

    // TODO COMPLETE HERE
    /**
     * @brief getImageFromArray
     * @param mapArrays
     * @param fromPgm
     * @return QImage
     * returns a QImage constructed from a QByteArray
     */
    QImage getImageFromArray(const QByteArray& mapArrays, const bool fromPgm);

signals:
    /// emitted when a user make an attempt at saving the map, the status holds the status regarding the operation (true = success, false = failure)
    void saveStatus(bool);

private:
    /**
     * @brief mapImage
     * The current displayed image
     */
    QImage mapImage;

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
    Position origin;
    /**
     * @brief rectangle
     * The rectangle whose corners are the extreme known points in each direction, its purpose is to
     * display the known part of the map by default
     */
    QRect rect;
    QPointF center;
    QDateTime dateTime;
    QUuid mapId;
    bool modified;
};

#endif /// MAP_H
