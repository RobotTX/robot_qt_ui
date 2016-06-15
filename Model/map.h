#ifndef MAP_H
#define MAP_H

#include "Model/position.h"
#include <QImage>

/**
 * @brief The Map class
 * The model class for the Map,
 * contains the map as a QImage and its width, height, resolution and the origin
 */
class Map{
public:
    Map();

    /// Getters
    QImage getMapImage(void) const { return mapImage; }
    float getResolution(void) const { return resolution; }
    int getWidth(void) const { return width; }
    int getHeight(void) const { return height; }
    Position getOrigin(void) const { return origin; }

    /// Setters
    void setResolution(const float _resolution) { resolution = _resolution; }
    void setWidth(const int _width) { width = _width; }
    void setHeight(const int _height) { height = _height; }
    void setOrigin(const Position _origin) { origin = _origin; }

    /**
     * @brief setMapFromArray
     * @param mapArrays
     * Create the QImage mapImage from an array of byte
     */
    void setMapFromArray(const QByteArray& mapArrays);

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
     * affecting the robot position)
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

};

#endif // MAP_H
