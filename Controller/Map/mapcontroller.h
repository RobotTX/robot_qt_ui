#ifndef MAPCONTROLLER_H
#define MAPCONTROLLER_H

#include <QObject>
#include <QPointer>
#include "Model/Map/map.h"
#include "Controller/Map/laserworker.h"

class EditMapController;
class QQmlApplicationEngine;
class MainController;
class ScanMapController;

class MapController : public QObject {

    Q_OBJECT

public:

    MapController(QQmlApplicationEngine* engine, QObject *applicationWindow, MainController *parent);

    /// Getters
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
    bool setMapFile(const QString file);
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
     * @param mapRotation
     * @return true if the configuration was successfully saved, false otherwise
     */
    bool saveMapConfig(const QString fileName, const double centerX, const double centerY, const double zoom, const int mapRotation) const;

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
     * @param mapRotation
     * Centers the map on (centerX, centerY) with a zoom coefficient of <zoom>
     */
    void centerMap(const double centerX, const double centerY, const double zoom, const int mapRotation);

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
     * to save the map file of the current configuration and
     * reload the correct map in the merge and edit windows
     */
    void saveNewMap(const QString file_name);
    /**
     * @brief updateMetadata
     * @param width
     * @param height
     * @param resolution
     * @param originX
     * @param originY
     * updates the metadata
     */
    void updateMetadata(int width, int height, double resolution, double originX, double originY);

private slots:
    /**
     * @brief loadPositionSlot
     * Restores the position of the map, the zoom and the rotation to the last configuration saved
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
     * @brief centerMapSlot
     * Emits the adequate position for the qml side to recenter the map
     */
    void centerMapSlot();

    /**
     * @brief SaveCenterMapSlot
     * Emits the adequate position for recording in currentMap.txt
     */
    void saveCenterMapPosSlot();

    /**
     * @brief savePositionSlot
     * @param posX
     * @param posY
     * @param zoom
     * @param mapRotation
     * @param mapSrc path of the pgm-format map file
     * Saves the current configuration inside the currentMap.txt configuration file
     */
    void savePositionSlot(const double posX, const double posY, const double zoom, const int mapRotation, const QString mapSrc);
    void savePositionSlot2(const double posX, const double posY, const double zoom, const int mapRotation, const QString mapSrc);
    /**
     * @brief saveEditedImage
     * @param location
     * Save the edited image and change it on the main window
     */
    void saveEditedImage(const QString location, int mapRotation);

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
     * @param mapRotation
     * Notifies qml to center the map on posX, posY with a zoom coefficient of <zoom>
     */
    void setMapPosition(QVariant posX, QVariant posY, QVariant zoom, QVariant mapRotation);

    void setRotation(QVariant mapRotation);

    /**
     * @brief requestReloadMap
     * @param location
     * Change the main window image to the given one
     */
    void requestReloadMap(QVariant location);
    /**
     * @brief sendMapToRobots
     * After saving the edited map we ask the robots controller to send it to everyone
     */
    void sendMapToRobots(QString mapId, QString date, QString mapMetadata, QImage img);
    /**
     * @brief centerPosition
     * The position of the center of the image in image coordinates
     */
    void centerPosition(QVariant, QVariant);

    /**
     * @brief saveCenterMapPos
     * Save center map position to local file
     */
    void saveCenterMapPos();

 private:
    QPointer<Map> map;
    QPointer<EditMapController> editMapController;
    QPointer<ScanMapController> scanMapController;
};

#endif /// MAPCONTROLLER_H
