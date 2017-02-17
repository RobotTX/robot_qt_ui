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
class RobotPositionRecovery;
class RobotsController;
class PathPointCreationWidget;

#include <QMainWindow>
#include <QModelIndex>
#include <QPair>
#include <QMessageBox>
#include <QSettings>
#include <QUuid>
#include <ctime>
#include "Controller/Points/pointscontroller.h"
#include "Model/Paths/paths.h"
#include "Model/Points/points.h"
#include "Model/Points/point.h"
#include "Model/Other/graphicitemstate.h"
#include "View/TopLayout/toplayoutwidget.h"
#include "View/Robots/robotview.h"

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

    void hideAllWidgets();
    void clearNewMap();
    void resetFocus();
    void updateAllPaths(const Point &old_point, const Point &new_point);
    void clearPath(const int robotNb);

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
    void updatePathPainter(QSharedPointer<Points>, bool);
    void updatePathPainterPointView(QSharedPointer<QVector<QSharedPointer<PointView>>>);
    void resetPath(QSharedPointer<Points>);
    void resetPathCreationWidget();
    void cancelRobotModifications();
    void receivedMapToMerge(QString, QImage, double, double, double);
    void receivedScanMap(QString, QImage, double);
    void startedScanning(QString, bool);
    void robotDisconnected(QString);
    void robotReconnected(QString);
    void robotScanning(bool, QString, bool);
    void robotRecovering(bool, QString, bool);
    void newBatteryLevel(int);
    void updatePath(const QString groupName, const QString pathName);
    void stopAllCmd();
    void tutorialSignal(const bool, const QString);
    void startedRecovering(QString, bool);
    void setMessageTop(QString,QString);
    void setTemporaryMessageTop(QString type, QString message, int ms);
    void enableTopLayout(bool);

public slots:
    void setGraphicItemsState(const GraphicItemState state);
    void switchFocus(const QString name, QWidget* widget, const MainWindow::WidgetType type);
    void setEnableAll(bool enable, GraphicItemState state = GraphicItemState::NO_STATE, int noReturn = -1);

private slots:
    void timerSlot();
    void sendPathSelectedRobotSlot(const QString groupName, const QString pathName);
    void mapReceivedSlot(const QByteArray mapArray, int who, QString mapId, QString mapDate, QString resolution, QString originX, QString originY, QString ipAddress);
    void startScanningSlot(QString robotName);
    void stopScanningSlot(QStringList listRobot);
    void playScanSlot(bool scan, QString robotName);
    void playRecoverySlot(bool recover, QString robotName);
    void robotGoToSlot(QString robotName, double x, double y);
    void saveScanMapSlot(double resolution, Position origin, QImage image, QString fileName);
    void teleopCmdSlot(QString robotName, int id);
    void setSelectedRobot(QPointer<RobotView> robotView);
    void setSelectedRobot(QAbstractButton* button);
    void setSelectedRobotNoParent(QAbstractButton *button);
    void setSelectedRobotFromPointSlot(QString robotName);
    void robotBtnEvent(void);
    void pointBtnEvent(void);
    void mapBtnEvent(void);
    void pathBtnEvent(void);
    void openLeftMenu(void);
    void editRobotBtnEvent(void);
    void checkRobotBtnEventMenu();
    void saveRobotModifications(void);
    void setSelectedTmpPoint(void);
    void pointSavedEvent(QString groupName, double x, double y, QString name);
    void deletePath(int robotNb);
    void playSelectedRobot(int robotNb);
    void savePathSlot(void);
    void editTmpPathPointSlot(int id, QString name, double x, double y);
    void saveEditPathPointSlot(void);
    void cancelEditPathPointSlot(void);
    void moveEditedPathPointSlot(void);
    void doubleClickOnPathsGroup(QString checkedButton);
    void enableReturnAndCloseButtons(void);
    void doubleClickOnRobotSlot(QString robotName);
    void setMessageCreationPath(QString message);
    void updateEditedPathPoint(double x, double y);
    void centerMap(void);
    void setMessageCreationPoint(QString type, PointsController::PointNameError error);
    void viewPathSelectedRobot(int robotNb, bool checked);
    void closeSlot();
    void showHome();
    void showAllHomes(void);
    void backEvent();
    void robotIsAliveSlot(QString hostname, QString ip, QString ssid, int stage, int battery);
    void robotIsDeadSlot(QString hostname, QString ip);
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
    void modifyGroupPathsWithEnter(QString name);
    void displayPath();
    void createPath();
    void deletePath();
    void displayPathOnMap(const bool display);
    void editPath();
    void doubleClickOnPath(QString pathName);
    void cancelNoRobotPathSlot();
    void displayAssignedPath(QString groupName, QString pathName);
    /// to clear paths on the map
    void clearMapOfPaths();

    void saveMapBtnEvent();
    void loadMapBtnEvent();

    void testCoordSlot(double x, double y);
    void editMapSlot();
    void updateRobotInfo(QString robot_name, QString robotInfo);
    void saveEditMapSlot();
    void mergeMapSlot();
    void saveMergeMapSlot(double resolution, Position origin, QImage image, QString fileName);
    void startRecoveringSlot(QString robotName);
    void stopRecoveringRobotsSlot(QStringList robotList);

    void testFunctionSlot();
    void activateLaserSlot(QString name, bool);
    void getMapForMergingSlot(QString robotName);
    void scanMapSlot();

    void updateBatteryLevel(const int level);
    void updateLaserSlot();
    void commandDoneSlot(QString cmdName, bool success, QString robotName, QString newRobotName, QString groupName, QString pathName, bool scan, int nb, QStringList path);

    /// sends a signal to the settings controller in order to keep track of the messages we need to show the user
    void relayTutorialSignal(const bool messageNeeded);
    void setMessageCreationPathTimerSlot();

    void resetPathSlot();
    void addPathPointSlot(QString name, double x, double y, int waitTime);
    void setCurrentPathSlot(const QVector<QSharedPointer<PathPoint>>& currentPath, QString pathName);
    void updatePathPainterSlot(bool savePath);
    void editPathPointSlot(int id, QString name, double, double);
    void deletePathPointSlot(int id);
    void orderPathPointChangedSlot(int from, int to);
    void updatePointsListSlot();
    void pointClicked(QAction* action);
    void editPathPointSlot();
    void updatePathPointCreationWidgetSlot(PathPointCreationWidget* pathPointCreationWidget);

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
    void setHomeAtConnection(const QString robot_name, const Position& pos_home);

private:
    Ui::MainWindow* ui;


    QVBoxLayout* rightLayout;
    BottomLayout* bottomLayout;


    QMessageBox msgBox;

    QList<QPair<QPair<QWidget*, QString>, MainWindow::WidgetType>> lastWidgets;

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
