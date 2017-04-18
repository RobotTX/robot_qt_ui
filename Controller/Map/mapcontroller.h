#ifndef MAPCONTROLLER_H
#define MAPCONTROLLER_H

#include <QObject>
#include <QPointer>
#include "Model/Map/map.h"

class EditMapController;
class QQmlApplicationEngine;
class MergeMapController;
class MainController;
class ScanMapController;

class MapController : public QObject {

    Q_OBJECT

public:

    MapController(QQmlApplicationEngine* engine, QObject *applicationWindow, MainController *parent);

    /// Getters
    QPointer<MergeMapController> getMergeMapController(void) const { return mergeMapController; }
    QPointer<ScanMapController> getScanMapController(void) const { return scanMapController; }

    QString getMapFile(void) const { return map->getMapFile(); }
    QImage getMapImage(void) const { return map->getMapImage(); }
    QUuid getMapId(void) const { return map->getMapId(); }
    QDateTime getDateTime(void) const { return map->getDateTime(); }
    int getHeight(void) const { return map->getHeight(); }
    int getWidth(void) const { return map->getWidth(); }
    QPointF getOrigin(void) const { return map->getOrigin(); }
    double getResolution(void) const { return map->getResolution(); }
    QString getMetadataString(void) const;

    /// Setters
    void setMapFile(const QString file);
    void setOrigin(const QPointF origin) { map->setOrigin(origin); }
    void setHeight(const int height) { map->setHeight(height); }
    void setWidth(const int width) { map->setWidth(width); }
    void setResolution(const double resolution) { map->setResolution(resolution); }
    void setMapId(const QUuid mapId) { map->setMapId(mapId); }
    void setDateTime(const QDateTime dateTime) { map->setDateTime(dateTime); }

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
    bool saveMapConfig(const QString fileName, const double centerX, const double centerY, const double zoom) const;

    /**
     * @brief loadMapConfig
     * @param fileName configuration file
     * @return true if the map could be loaded and false otherwise
     * Imports the map with the configuration given in the configuration file
     */
    bool loadMapConfig(const QString fileName);

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
    void centerMap(const double centerX, const double centerY, const double zoom);

    /**
     * @brief getImageFromArray
     * @param mapArrays
     * @param map_width
     * @param map_height
     * @param fromPgm
     * returns a QImage constructed from a QByteArray
     */
    QImage getImageFromArray(const QByteArray& mapArrays, const int map_width, const int map_height, const bool fromPgm);

    /**
     * @brief newMapFromRobot
     * @param mapArray
     * @param mapId
     * @param mapDate
     * Save the image we just received from a robot as the currentMap
     * <fromPgm> typically is false if the map comes from the robot, true if it was stored in the file system
     */
    void newMapFromRobot(const QByteArray& mapArray, const QString mapId, const QString mapDate);
    /**
     * @brief saveNewMap
     * @param file_name
     * to save the map of the current configuration
     */
    void saveNewMap(const QString file_name);

private slots:
    /**
     * @brief loadPositionSlot
     * Restores the position of the map (and zoom) to the last configuration saved
     */
    void loadPositionSlot();

    /**
     * @brief posClicked
     * @param x
     * @param y
     * Display the clicked position in robot and map coordinates when we click on the map
     */
    void posClicked(const double x, const double y);

public slots:
    /**
     * @brief savePositionSlot
     * @param posX
     * @param posY
     * @param zoom
     * @param mapSrc path of the pgm-format map file
     * Saves the current configuration inside the currentMap.txt configuration file
     */
    void savePositionSlot(const double posX, const double posY, const double zoom, const QString mapSrc);

    /**
     * @brief saveEditedImage
     * @param location
     * Save the edited image and change it on the main window
     */
    void saveEditedImage(const QString location);

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

    /**
     * @brief requestReloadMap
     * @param location
     * Change the main window image to the given one
     */
    void requestReloadMap(QVariant location);

 private:
    QPointer<Map> map;
    QPointer<MergeMapController> mergeMapController;
    QPointer<EditMapController> editMapController;
    QPointer<ScanMapController> scanMapController;
};

#endif /// MAPCONTROLLER_H
