#ifndef MAINWINDOW_H
#define MAINWINDOW_H

class RobotView;
class PointView;
class CustomQGraphicsView;
class Map;
class LeftMenuWidget;
class RobotsLeftWidget;
class MapLeftWidget;
class EditSelectedRobotWidget;
class LeftMenu;
class BottomLayout;
class PathCreationWidget;
class QAbstractButton;
class QVBoxLayout;
class PathPainter;
class CustomPushButton;
class GroupsPathsWidget;
class CommandController;
class EditMapWidget;
class MergeMapWidget;
class SettingsWidget;
class ScanMapWidget;
class DrawObstacles;
class MapView;
class LaserController;
class SettingsController;
class TopLayoutController;
class MapController;
class PathsController;
class PointsController;
class RobotPositionRecovery;
class RobotsController;

#include "Model/paths.h"
#include "View/createpointwidget.h"
#include "View/toplayoutwidget.h"
#include "Model/points.h"
#include "View/robotview.h"
#include <QMainWindow>
#include <QModelIndex>
#include "Model/graphicitemstate.h"
#include <QPair>
#include <QMessageBox>
#include "Model/point.h"
#include <QSettings>
#include <QUuid>
#include <ctime>

#define DESKTOP_PATH "/home/m-a/Desktop/"
#define GOBOT_PATH "/home/m-a/Documents/QtProject/gobot-software/"

#define XML_FILE "points.xml"
#define ROBOTS_NAME_FILE "robotsName.dat"
#define MAP_FILE "realMap.dat"
#define PATHS_FILE "savedPaths.dat"

#define PI 3.14159265
#define PORT_ROBOT_UPDATE 6000

namespace Ui {
    class MainWindow;
}

/**
 * @brief MainWindow::MainWindow
 * @param parent
 * The main controller of the application
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    enum WidgetType { MENU, GROUPS, GROUP, POINT, ROBOTS, ROBOT , MAP, GROUPS_PATHS, GROUP_OF_PATHS, PATH };

    QList<QPair<QPair<QWidget*, QString>, MainWindow::WidgetType>> getLastWidgets(void) const { return lastWidgets; }
    CommandController* getCommandController(void) const { return commandController; }
    LeftMenu* getLeftMenu(void) const { return leftMenu; }
    LaserController* getLaserController(void) const { return laserController; }
    MapController* getMapController(void) const { return mapController; }
    PathsController* getPathsController(void) const { return pathsController; }
    PointsController* getPointsController(void) const { return pointsController; }
    RobotsController* getRobotsController(void) const { return robotsController; }
    TopLayoutController* getTopLayoutController(void) const { return topLayoutController; }

    void initializeMenu();
    void initializeBottomPanel();
    void initializeLeftMenu();
    void hideAllWidgets();
    int openConfirmMessage(const QString);
    void clearNewMap();
    void resetFocus();
    /// to sleep for ms milliseconds
    static void delay(const int ms);
    void updateAllPaths(const Point &old_point, const Point &new_point);
    void clearPath(const int robotNb);

    /// returns true if the first date is later to the second date
    bool isLater(const QStringList& date, const QStringList& otherDate);

    QPair<Position, QStringList> getHomeFromFile(const QString robot_name);
    QPair<QPair<QString, QString>, QStringList> getPathFromFile(const QString robot_name);

    QVector<PathPoint> extractPathFromInfo(const QStringList& robotInfo);

    void updateHomeInfo(const QString robot_name, QString posX, QString posY, QString homeDate);
    void updatePathInfo(const QString robot_name, QString pathDate, QStringList path);
    void updateMapInfo(const QString robot_name, QString mapId, QString mapDate);

    QString prepareCommandPath(const Paths::Path& path) const;
    void saveMap(QString fileName);

    void openHelpMessage(const QString message, const QString feature);

    void openPositionRecoveryWidget(void);

signals:
    void changeCmdThreadRobotName(QString);
    void addPathPoint(QString name, double x, double y);
    void addNoRobotPathPoint(QString name, double x, double y);
    void updatePathPainter(bool);
    void updatePathPainterPointView();
    void resetPath();
    void resetPathCreationWidget();
    void cancelRobotModifications();
    void receivedMapToMerge(QString, QImage, double, double, double);
    void receivedScanMap(QString, QImage, double);
    void startedScanning(QString, bool);
    void robotDisconnected(QString);
    void robotReconnected(QString);
    void robotScanning(bool, QString, bool);
    void newBatteryLevel(int);
    void updatePath(const QString groupName, const QString pathName);
    void stopAllCmd();
    void tutorialSignal(const bool, const QString);

public slots:
    void setGraphicItemsState(const GraphicItemState state);
    void switchFocus(const QString name, QWidget* widget, const MainWindow::WidgetType type);
    void setEnableAll(bool enable, GraphicItemState state = GraphicItemState::NO_STATE, int noReturn = -1);

private slots:
    void setTemporaryMessageTop(const QString type, const QString message, const int ms);
    void sendPathSelectedRobotSlot(const QString groupName, const QString pathName);
    void updateMetadata(const int width, const int height, const float resolution, const float originX, const float originY);
    void mapReceivedSlot(const QByteArray mapArray, int who, QString mapId, QString mapDate, QString resolution, QString originX, QString originY, QString ipAddress);
    void startScanningSlot(QString robotName);
    void stopScanningSlot(QStringList listRobot);
    void playScanSlot(bool scan, QString robotName);
    void robotGoToSlot(QString robotName, double x, double y);
    void saveScanMapSlot(double resolution, Position origin, QImage image, QString fileName);
    void teleopCmdSlot(QString robotName, int id);
    void quit(void);
    void setSelectedRobot(QPointer<RobotView> robotView);
    void setSelectedRobot(QAbstractButton* button);
    void setSelectedRobotNoParent(QAbstractButton *button);
    void setSelectedRobotFromPointSlot(QString robotName);
    void robotBtnEvent(void);
    void pointBtnEvent(void);
    void mapBtnEvent(void);
    void pathBtnEvent(void);
    void openLeftMenu(void);
    void backRobotBtnEvent(void);
    void editRobotBtnEvent(void);
    void checkRobotBtnEventMenu();
    void checkRobotBtnEvent(QString name);
    void cancelEditSelecRobotBtnEvent(void);
    void saveRobotModifications(void);
    void setSelectedTmpPoint(void);
    void pointSavedEvent(QString groupName, double x, double y, QString name);
    void deletePath(int robotNb);
    void playSelectedRobot(int robotNb);
    void savePathSlot(void);
    void addPointPathSlot(QString name, double x, double y, GraphicItemState);
    void editTmpPathPointSlot(int id, QString name, double x, double y);
    void saveEditPathPointSlot(void);
    void cancelEditPathPointSlot(void);
    void moveEditedPathPointSlot(void);
    void doubleClickOnPathsGroup(QString checkedButton);
    void enableReturnAndCloseButtons(void);
    void doubleClickOnRobot(QString checkedId);
    void setMessageCreationPath(QString message);
    void updateEditedPathPoint(double x, double y);
    void centerMap(void);
    void setMessageCreationPoint(QString type, CreatePointWidget::Error error);
    void viewPathSelectedRobot(int robotNb, bool checked);
    void closeSlot();
    void showHome();
    void showEditHome();
    void showAllHomes(void);
    void backEvent();
    void updateView();
    void robotIsAliveSlot(QString hostname, QString ip, QString ssid, int stage, int battery);
    void robotIsDeadSlot(QString hostname, QString ip);
    void selectViewRobot();
    void sendNewMapToRobots(QString ipAddress = "");
    void settingBtnSlot();
    void updatePathPainterPointViewSlot();
    void stopPath(int robotNb);
    void resetPathPointViewsSlot();
    void deletePathSelecRobotBtnEvent();
    void deletePathSlot(QString groupName, QString pathName);
    void editPathSlot(QString groupName, QString pathName);
    void setNewHome(QString homeName);
    void goHome();
    void goHome(int nbRobot);

    /// for menu paths
    void displayGroupPaths();
    void createGroupPaths();
    void deleteGroupPaths();
    void saveGroupPaths(QString name);
    void modifyGroupPathsWithEnter(QString name);
    void displayPath();
    void createPath();
    void deletePath();
    void displayPathOnMap(const bool display);
    void editPath();
    void doubleClickOnPath(QString pathName);
    void setMessageNoRobotPath(const int code);
    void cancelNoRobotPathSlot();
    void saveNoRobotPathSlot();
    void setMessageModifGroupPaths(int code);
    void displayAssignedPath(QString groupName, QString pathName);
    /// to clear paths on the map
    void clearMapOfPaths();

    void messageMapSaved(bool status);
    void saveMapBtnEvent();
    void loadMapBtnEvent();

    void testCoordSlot(double x, double y);
    void editMapSlot();
    void updateRobotInfo(QString robot_name, QString robotInfo);
    void setHomeAtConnection(const QString robot_name, const Position& pos_home);
    bool updateHomeFile(const QString robot_name, const Position& robot_home_position, const QStringList date);
    void saveEditMapSlot();
    void mergeMapSlot();
    void saveMergeMapSlot(double resolution, Position origin, QImage image, QString fileName);

    void testFunctionSlot();
    void activateLaserSlot(QString name, bool);
    void getMapForMergingSlot(QString robotName);
    void scanMapSlot();

    void updateBatteryLevel(const int level);
    void updateLaserSlot();
    void commandDoneSlot(QString cmdName, bool success, QString robotName, QString newRobotName, QString groupName, QString pathName, bool scan, int nb, QStringList path);

    /// sends a signal to the settings controller in order to keep track of the messages we need to show the user
    void relayTutorialSignal(const bool messageNeeded);

protected:
    void closeEvent(QCloseEvent *event);
    void robotHasNoHome(QString robotName);
    void closeWidgets();
    void commandDoneNewName(bool success, QString name);
    void commandDonePausePath(bool success, QString robotName);
    void commandDonePlayScan(bool success, bool scan, QString robotName);
    void commandDonePauseScan(bool success, bool scan, QString robotName);
    void commandDoneSendMap(bool success);
    void commandDoneSendPorts(bool success);
    void commandDoneSendPath(bool success, bool boolean, QString robotName, QString groupName, QString pathName, QStringList path);
    void commandDonePlayPath(bool success, QString robotName);
    void commandDoneDeletePath(bool success, QString robotName);
    void commandDoneStopPath(bool success, QString robotName);
    void commandDoneStopDeletePath(bool success, QString robotName);
    void commandDoneNewHome(bool success, QString robotName, int id, QString homeName);
    void commandDoneGoHome(bool success, QString robotName);
    void commandDoneStopGoHome(bool success);
    void commandDoneStartLaser(bool success);
    void commandDoneStopLaser(bool success);
    void commandDoneReceiveMap(bool success);
    void commandDoneStartScan(bool success, bool scan, QString robotName);
    void commandDoneStopScan(bool success, QString robotName);

private:
    Ui::MainWindow* ui;


    QVBoxLayout* rightLayout;
    BottomLayout* bottomLayout;


    QMessageBox msgBox;

    QList<QPair<QPair<QWidget*, QString>, MainWindow::WidgetType>> lastWidgets;

    LeftMenuWidget* leftMenuWidget;
    MapLeftWidget* mapLeftWidget;
    QPointer<EditMapWidget> editMapWidget;
    QPointer<MergeMapWidget> mergeMapWidget;
    QPointer<ScanMapWidget> scanMapWidget;
    QPointer<RobotPositionRecovery> robotPositionRecoveryWidget;

    LeftMenu* leftMenu;

    /// feature that we are currently using, useful to determine which tutorial message to enable / disable
    QString currentFeature;

    TopLayoutController* topLayoutController;
    MapController* mapController;
    CommandController* commandController;
    SettingsController* settingsController;
    LaserController* laserController;
    PathsController* pathsController;
    PointsController* pointsController;
    RobotsController* robotsController;
};

#endif /// MAINWINDOW_H
