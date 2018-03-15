#ifndef MAINCONTROLLER_H
#define MAINCONTROLLER_H

class QQmlApplicationEngine;
class MainMenuController;
class MapController;
class PointController;
class SpeechController;
class PathController;
class RobotsController;

#include <QObject>
#include <QList>
#include <QByteArray>
#include <QPointer>
#include <QImage>
#include <QDir>

class MainController : public QObject {

    Q_OBJECT

public:

    MainController(QQmlApplicationEngine* _engine, QObject* parent = Q_NULLPTR);

    QPointer<MapController> getMapController(void) const { return mapController; }
    QPointer<RobotsController> getRobotsController(void) const { return robotsController; }

private :
    /**
     * @brief sendNewMap
     * @param ip
     * sends the image used by the map controller to the robot at ip <ip>
     */
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
      * @brief checkSpeech
      * @param name
      * To check if the speech <name> is okay
      */

     void checkSpeech(QString name, QString oldName);

    /**
     * @brief saveMapConfig
     * @param fileName the map file name (.pgm)
     * @param zoom
     * @param centerX
     * @param centerY
     * @param mapRotation
     * To export the current map in files (one for the image, one for the configuration, one for the points, one for the paths)
     */
    void saveMapConfig(QString fileName, double zoom, double centerX, double centerY, int mapRotation, bool new_config = false) const;
    /**
     * @brief loadMapConfig
     * @param fileName configuration file
     * To import a map inside the application (along with its points, configuration and paths)
     */
    void loadMapConfig(QString fileName);
    /**
     * @brief checkTmpPosition
     * @param index
     * @param x
     * @param y
     * Check the position of the path point at <index> when creating a path
     */
    void checkTmpPosition(int index, double x, double y);
    /**
     * @brief newRobotPosSlot
     * @param ip
     * @param posX
     * @param posY
     * @param ori
     * updates the position of the robot (on the map and in the scan window if necessary)
     * with a new position (posX, posY) and orientation <ori>
     */
    void newRobotPosSlot(QString ip, double posX, double posY, double ori);
    /**
     * @brief updatePathSlot
     * @param ip
     * @param strList
     * updates the path of the robot at ip <ip>
     * <strList> contains the path points
     */
    void updatePathSlot(QString ip, QStringList strList);
    /**
     * @brief updateHomeSlot
     * @param ip
     * @param homeX
     * @param homeY
     * updates the model qml with a new home for the robot whose ip is <ip>
     */
    void updateHomeSlot(QString ip, double homeX, double homeY, double homeOri);
    /**
     * @brief sendCommandNewHome
     * @param ip
     * @param homeName
     * @param homeX
     * @param homeY
     * sends a new home to the robot at ip <ip>
     */

    void updateLinearVelocitySlot(QString ip, double linear);

    void sendCommandNewHome(QString ip, double homeX, double homeY, int homeOri);
    /**
     * @brief sendCommandNewPath
     * @param ip
     * @param groupName
     * @param pathName
     * send a new path to the robot at ip <ip> given as a groupName and a pathName
     */
    void sendCommandNewPath(QString ip, QString groupName, QString pathName);
    /**
     * @brief checkMapInfoSlot
     * @param ip
     * @param mapId
     * @param mapDate
     * compares the map of the application and the map of the robot and
     * ask the user to make a choice in case they differ
     */
    void checkMapInfoSlot(QString ip, QString mapId, QString mapDate);
    /**
     * @brief saveSettings
     * @param mapChoice
     * @param batteryThreshold
     * @param showTutorial
     * saves the settings into a file, which map to choose if app and robot
     * have a different map, the level of battery under which the user is warned
     * and whether or not the tutorial should be shown to the user
     */
    void saveSettings(int mapChoice, double batteryThreshold);
    /**
     * @brief newMapFromRobotSlot
     * @param ip
     * @param mapArray
     * @param mapId
     * @param mapDate
     * updates the map of the map controller, resets paths and points and sends
     * the map to all the other robots
     */
    void newMapFromRobotSlot(QString ip, QByteArray mapArray, QString mapId, QString mapDate, QString resolution, QString originX, QString originY, int map_width, int map_height);
    /**
     * @brief requestOrSendMap
     * @param ip
     * @param request
     * request or send map to robot at ip <ip> depending on <request>
     */
    void requestOrSendMap(QString ip, bool request);
    /**
     * @brief getMapFromRobot
     * @param ip
     * actually used to get the map from the robot when trying to merge maps
     * within the merge maps window
     */
    void getMapFromRobot(QString ip);
    /**
     * @brief processMapForMerge
     * @param mapArray
     * @param resolution
     * converts a QByteArray to a QImage to be merged after
     */
    /**
     * @brief startScanningSlot
     * @param ip
     * sends a command to the robot to start scanning the map
     */
    void startScanningSlot(QString ip);
    /**
     * @brief startAutomaticScanSlot
     * @param ip
     * sends a command to the robot at ip <ip> to start scanning automatically
     */
    void startAutomaticScanSlot(QString ip);
    /**
     * @brief stopScanningSlot
     * @param ip
     * sends a command to the robot at ip <ip> to stop scanning the map
     */
    void stopScanningSlot(QString ip, bool killGobotMove);
    /**
     * @brief playPauseScanningSlot
     * @param ip
     * @param wasScanning
     * @param scanningOnConnection
     * plays or pauses the scan, depending on whether it was already scanning
     * (connection problem for example), the user is asked if he wants to start over or not
     */
    void playPauseScanningSlot(QString ip, bool wasScanning, bool scanningOnConnection);
    /**
     * @brief MainController::playPauseExploringSlot
     * @param ip
     * @param wasExploring
     * play or pause the automatic exploration of the map while scanning
     */
    void playPauseExploringSlot(QString ip, bool wasExploring);
    /**
     * @brief receivedScanMapSlot
     * @param ip
     * @param map
     * @param resolution
     * gives the newly received scan map to the scan controller
     */
    void receivedScanMapSlot(QString ip, QByteArray map, QString resolution, QString originX, QString originY, int map_width, int map_height);
    /**
     * @brief sendTeleopSlot
     * @param ip
     * @param teleop
     * relays the teleop command to the robots controller
     */
    void sendTeleopSlot(QString ip, int teleop);
    /**
     * @brief resetMapConfiguration
     * @param file_name
     * after a merged map has been saved, it replaces the main window map
     * paths and points are cleared, the new configuration is saved
     * if
     */
    void resetMapConfiguration(QString file_name, bool scan, double centerX = 0.0, double centerY = 0.0);
    /**
     * @brief removeScanMapSlot
     * @param ip
     * get the scan controller to remove the scan map
     * of the robot at ip <ip>
     */
    void removeScanMapSlot(QString ip);
    /**
     * @brief sendScanGoal
     * @param ip
     * @param x
     * @param y
     * gets the robots controller to send a goal position (x, y)
     * to the robot at ip <ip>
     */
    void sendScanGoal(QString ip, double x, double y);
    /**
     * @brief setMessageTopSlot
     * @param status
     * @param msg
     * notifies the qml side to put the msg <msg>
     * with the status <status> at the top of the main window
     * of the application
     */
    void setMessageTopSlot(int status, QString msg);
    /**
     * @brief updateTutoFile
     * @param index
     * @param visible
     * needs to replace the index'th value by <visible> in tutorial.txt
     */
    void updateTutoFile(int index, bool visible);
    /**
     * @brief clearPointsAndPathsAfterScan
     * A new scanned map is now used in the application
     * so points and paths must be cleared
     */
    void clearPointsAndPathsAfterScan(void);
    /**
     * @brief setDiscardMap
     * @param discard
     * To discard any new scan map after the scan has finished
     */
    void setDiscardMap(bool discard){ discardMap = discard; }
    /**
     * @brief sendMapToAllRobots
     * @param id
     * @param date
     * @param metadata
     * @param img
     * Orders the robots controller to send the described map to all robots
     * This is called by the map controller when an edited map is saved
     */
    void sendMapToAllRobots(QString id, QString date, QString metadata, QImage img);

    void testSlot();

    void deconnexionSlot();

    void saveWifi(QString ip, QString wifi, QString pwd);

    void sendCommandSavePlace(QString, QString, double, double, double, bool);

    void sendCommandTtsToRobot(QString ip, QString tts);

    void saveVelocity(QString ip, double linearVelocity, double angularVelocity);

    void saveBattery(QString ip, double battery);

signals:
    /// those signals are connected to the qml model to keep the data consistent between the c++ side and the qml side
    void setHome(QVariant ip, QVariant posX, QVariant posY, QVariant homeOri);
    void setLinearVelocity(QVariant ip, QVariant linear);
    void setPath(QVariant ip, QVariant name);
    void addPathPoint(QVariant ip, QVariant name, QVariant posX, QVariant posY, QVariant waitTime, QVariant orientation);
    void emitSettings(QVariant mapChoice);
    void openMapChoiceMessageDialog(QVariant ip, QVariant robotIsOlder);
    void openRestartScanMessageDialog(QVariant ip);
    void emitBatteryThreshold(QVariant batteryThreshold);
    void setMessageTop(QVariant status, QVariant msg);
    void deco();
    /**
     * @brief sendImageToMerge
     * @param resolution
     * sends the map to the merge map controller
     */
    void openWarningDialog(QVariant title, QVariant msg);
    /**
     * @brief updateTutorialMessageVisibility
     * @param feature
     * @param visible
     * updates the visibility of the message for the feature <feature>
     */
    void updateTutorialMessageVisibility(QVariant feature, QVariant visible);
    void updateRobotPos(QString ip, double x, double y, double orientation);

    void openScanWindowForAutomaticScan(QVariant ip);

private:
    QPointer<MainMenuController> mainMenuController;
    QPointer<MapController> mapController;
    QPointer<PointController> pointController;
    QPointer<SpeechController> speechController;
    QPointer<PathController> pathController;
    QPointer<RobotsController> robotsController;

    /**
     * @brief discardMap
     * Boolean to tell that even if we received a new map from a scan, we already saved the map and don't want to use the new one
     * and don't want to update the metadata
     */
    bool discardMap;
};

#endif /// MAINCONTROLLER_H
