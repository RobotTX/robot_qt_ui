#ifndef MAINCONTROLLER_H
#define MAINCONTROLLER_H

class QQmlApplicationEngine;
class MainMenuController;
class MapController;
class PointController;
class PathController;
class RobotsController;

#include <QObject>
#include <QList>
#include <QByteArray>
#include <QPointer>
#include <QImage>

class MainController : public QObject {

    Q_OBJECT

public:

    MainController(QQmlApplicationEngine* _engine, QObject* parent = Q_NULLPTR);

private :
    void sendNewMap(QString ip);

private slots:
    /**
     * @brief checkPoint
     * @param name
     * @param x
     * @param y
     * To check if the point <name> located at (x, y) is within the known area of the map
     */
     void checkPoint(QString name, QString oldName, double x, double y);

    /**
     * @brief saveMapConfig
     * @param fileName the map file name (.pgm)
     * @param zoom
     * @param centerX
     * @param centerY
     * To export the current map in files (one for the image, one for the configuration, one for the points, one for the paths)
     */
    void saveMapConfig(QString fileName, double zoom, double centerX, double centerY) const;
    /**
     * @brief loadMapConfig
     * @param fileName configuration file
     * To import a map inside the application (along with its points, configuration and paths)
     */
    void loadMapConfig(QString fileName) const;

    void checkTmpPosition(int index, double x, double y);
    void newRobotPosSlot(QString ip, float posX, float posY, float ori);
    void newMetadataSlot(int width, int height, float resolution, float originX, float originY);
    void updatePathSlot(QString ip, QStringList strList);
    void updateHomeSlot(QString ip, QString homeName, float homeX, float homeY);
    void sendCommandNewHome(QString ip, QString homeName, double homeX, double homeY);
    void sendCommandNewPath(QString ip, QString groupName, QString pathName);
    void checkMapInfoSlot(QString ip, QString mapId, QString mapDate);
    void saveSettings(int mapChoice, double batteryThreshold, bool showTutorial);
    void newMapFromRobotSlot(QString ip, QByteArray mapArray, QString mapId, QString mapDate);
    void requestOrSendMap(QString ip, bool request);

    void getMapFromRobot(QString ip);
    void processMapForMerge(QByteArray mapArray, QString resolution);

    void startScanningSlot(QString ip);
    void stopScanningSlot(QString ip);
    void playPauseScanningSlot(QString ip, bool wasScanning, bool scanningOnConnection);
    void receivedScanMapSlot(QString ip, QByteArray map, QString resolution);
    void sendTeleopSlot(QString ip, int teleop);

    void resetMapConfigurationAfterMerge(QString file_name);

    void removeScanMapSlot(QString ip);
    void sendScanGoal(QString ip, double x, double y);
    void setMessageTopSlot(int status, QString msg);

signals:
    void setHome(QVariant ip, QVariant name, QVariant posX, QVariant posY);
    void setPath(QVariant ip, QVariant name);
    void addPathPoint(QVariant ip, QVariant name, QVariant posX, QVariant posY, QVariant waitTime);
    void emitSettings(QVariant mapChoice, QVariant showTutorial);
    void openMapChoiceMessageDialog(QVariant ip, QVariant robotIsOlder);
    void sendImageToMerge(QImage, double resolution);
    void openRestartScanMessageDialog(QVariant ip);
    void emitBatteryThreshold(QVariant batteryThreshold);
    void setMessageTop(QVariant status, QVariant msg);

private:
    QPointer<MainMenuController> mainMenuController;
    QPointer<MapController> mapController;
    QPointer<PointController> pointController;
    QPointer<PathController> pathController;
    QPointer<RobotsController> robotsController;
};

#endif /// MAINCONTROLLER_H
