#ifndef MAPCONTROLLER_H
#define MAPCONTROLLER_H

#include <QObject>
#include "Model/Map/map.h"

class EditMapController;
class QQmlApplicationEngine;

class MapController : public QObject {

    Q_OBJECT

public:

    MapController(QQmlApplicationEngine* engine, QObject *applicationWindow, QObject* parent);

    QString getMapFile(void) const { return map->getMapFile(); }
    QImage getMapImage(void) const { return map->getMapImage(); }

    void setMapFile(const QString file) { map->setMapFile(file); }

    /**
     * @brief initializeMap
     * Initializes the map with the proper configuration given in currentMap.txt (build directory)
     */
    void initializeMap(void);

    /**
     * @brief saveMapConfig
     * @param fileName image file of the pgm-format map
     * @param centerX
     * @param centerY
     * @param zoom
     * @return true if the configuration was successfully saved, false otherwise
     */
    bool saveMapConfig(const std::string fileName, const double centerX, const double centerY, const double zoom) const;

    /**
     * @brief loadMapConfig
     * @param fileName configuration file
     * @return true if the map could be loaded and false otherwise
     * Imports the map with the configuration given in the configuration file
     */
    bool loadMapConfig(const std::string fileName);

    /**
     * @brief saveMapToFile
     * @param fileName
     * Saves the image on the computer and resets the <modified> attribute to false
     */
    void saveMapToFile(const QString fileName) const;

    /**
     * @brief centerMap
     * @param centerX
     * @param centerY
     * @param zoom
     * Centers the map on (centerX, centerY) with a zoom coefficient of <zoom>
     */
    void centerMap(double centerX, double centerY, double zoom);

private slots:
    /**
     * @brief loadPositionSlot
     * Restores the position of the map (and zoom) to the last configuration saved
     */
    void loadPositionSlot();

public slots:
    /**
     * @brief savePositionSlot
     * @param posX
     * @param posY
     * @param zoom
     * @param mapSrc pgm-format map file
     * Saves the current configuration inside the currentMap.txt configuration file
     */
    void savePositionSlot(double posX, double posY, double zoom, QString mapSrc);

    void saveEditedImage(QString location);

signals:
    /**
     * @brief setMap
     * @param mapSrc
     * Changes the view using the <mapSrc> file
     */
    void setMap(QVariant mapSrc);
    /**
     * @brief setMapPosition
     * @param posX
     * @param posY
     * @param zoom
     * Notifies qml to center the map on posX, posY with a zoom coefficient of <zoom>
     */
    void setMapPosition(QVariant posX, QVariant posY, QVariant zoom);
    void requestReloadMap(QVariant location);

private:
    Map* map;
    EditMapController* editMapController;
};

#endif /// MAPCONTROLLER_H
