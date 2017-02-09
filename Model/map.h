#ifndef MAP_H
#define MAP_H

#include "Model/position.h"
#include <QImage>
#include <QObject>
#include <QDateTime>
#include <QUuid>
#include <QSharedPointer>

/**
 * @brief The Map class
 * The model class for the Map,
 * contains the map as a QImage and its width, height, resolution and the origin
 */
class Map : public QObject {

    Q_OBJECT

public:

    explicit Map();

    QImage getMapImage(void) const { return mapImage; }
    float getResolution(void) const { return resolution; }
    int getWidth(void) const { return width; }
    int getHeight(void) const { return height; }
    Position getOrigin(void) const { return origin; }
    QRect getRect(void) const { return rect; }
    QDateTime getDateTime(void) const { return dateTime; }
    QUuid getMapId(void) const { return mapId; }
    bool getModified(void) const { return modified; }
    std::string getMapFile(void) const { return mapFile; }
    QPair<QPointF, float> getMapState(void) const { return mapState; }

    void setResolution(const float _resolution) { resolution = _resolution; }
    void setWidth(const int _width) { width = _width; }
    void setHeight(const int _height) { height = _height; }
    void setOrigin(const Position _origin) { origin = _origin; }
    void setMapImage(const QImage _mapImage) { mapImage = _mapImage; modified = true; }
    void setDateTime(const QDateTime _dateTime);
    void setMapId(const QUuid _mapId) { mapId = _mapId; }
    void setModified(const bool _modified) { modified = _modified; }
    void setZoomCoeff(const double coeff) { mapState.second = coeff; }
    void setMapPosition(const QPointF& pos) { mapState.first = pos; }
    void setMapFile(const std::string file) { mapFile = file; }
    void setCoeff(const float coeff) { mapState.second = coeff; }

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

    /**
     * @brief getImageFromArray
     * @param mapArrays
     * @param fromPgm
     * @return QImage
     * returns a QImage constructed from a QByteArray
     */
    QImage getImageFromArray(const QByteArray& mapArrays, const bool fromPgm);

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
     * @brief modified
     * holds whether the map has been modified since the last time it was saved
     */
    bool modified;

    /**
     * @brief mapFile
     * File where the currently used map is stored
     */
    std::string mapFile;

    /**
     * @brief mapState
     * The point is the point we put in the center of the scene
     * the float is the zoom coefficient
     */
    QPair<QPointF, float> mapState;
};

#endif /// MAP_H
