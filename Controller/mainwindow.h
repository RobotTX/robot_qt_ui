#ifndef MAINWINDOW_H
#define MAINWINDOW_H

class RobotView;
class PointView;
class ScanMapWorker;
class RobotServerWorker;
class CustomQGraphicsView;
class Map;
class Robots;
class MapView;
class LeftMenuWidget;
class PointsLeftWidget;
class RobotsLeftWidget;
class MapLeftWidget;
class EditSelectedRobotWidget;
class SelectedPointWidget;
class LeftMenu;
class BottomLayout;
class PathCreationWidget;
class QAbstractButton;
class QVBoxLayout;
class PathPainter;
class CustomPushButton;
class GroupsPathsWidget;
class QMoveEvent;
class CommandController;

#include "zipreader.h"
#include "zipwriter.h"

#include "Model/paths.h"
#include "View/createpointwidget.h"
#include "View/toplayout.h"
#include "Model/points.h"
#include "View/robotview.h"
#include <QMainWindow>
#include <QModelIndex>
#include "Model/graphicitemstate.h"
#include <QPair>
#include <QMessageBox>
#include "Model/point.h"
#include <QSettings>
#include <QThread>
#include <QUuid>

#include <ctime>

#define DESKTOP_PATH "/home/m-a/Desktop/"
#define GOBOT_PATH "/home/m-a/Documents/QtProject/gobot-software/"
/*
#define DESKTOP_PATH "/home/joan/Desktop/"
#define GOBOT_PATH "/home/joan/Gobot/gobot-software/"
*/
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
class MainWindow : public QMainWindow{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    enum WidgetType { MENU, GROUPS, GROUP, POINT, ROBOTS, ROBOT , MAP, GROUPS_PATHS, GROUP_OF_PATHS, PATH };

    QSharedPointer<Points> getPoints(void) const { return points; }
    QList<QPair<QPair<QWidget*, QString>, MainWindow::WidgetType>> getLastWidgets() const { return lastWidgets; }
    PathPainter* getPathPainter(void) const { return pathPainter; }
    CommandController* getCommandController() const { return commandController; }

    void initializeMenu();
    void initializeRobots();
    void initializePoints();
    void savePoints(const QString fileName);
    void initializeBottomPanel();
    void initializeLeftMenu();
    void initializePaths();
    void hideAllWidgets();
    int openConfirmMessage(const QString);
    void openInterdictionOfPointRemovalMessage(const QString pointName, const QString robotName);
    int openEmptyGroupMessage(const QString groupName);
    void clearNewMap();
    void resetFocus();
    void switchFocus(const QString name, QWidget* widget, const WidgetType type);
    /// to sleep for ms milliseconds
    static void delay(const int ms);
    void setTemporaryMessageTop(const QString type, const QString message, const int ms);
    void updateAllPaths(const Point &old_point, const Point &new_point);
    void clearPath(const int robotNb);
    QPointer<RobotView> getSelectedRobot(void) const { return selectedRobot; }
    QPointer<MapView> getMapView(void) const { return mapPixmapItem; }
    void serializePaths(const QString fileName);
    void deserializePaths(const QString fileName);
    void showHomes();
    void showHomes(QPointer<Robot> robot);
    void showSelectedRobotHomeOnly();
    void updateModelPaths(const Point &old_point, const Point &new_point);
    bool sendHomeToRobot(QPointer<RobotView> robot, QSharedPointer<PointView> home);

    void compress(const QString zipFile);
    void decompress(const QString fileName);

    bool saveMapConfig(const std::string fileName);
    bool loadMapConfig(const std::string fileName);

    /// returns true if the first date is later to the second date
    bool isLater(const QStringList date, const QStringList otherDate);

signals:
    void nameChanged(QString, QString);
    void changeCmdThreadRobotName(QString);
    void addPathPoint(QString name, double x, double y);
    void addNoRobotPathPoint(QString name, double x, double y);
    void updatePathPainter(bool);
    void updatePathPainterPointView();
    void resetPath();
    void resetPathCreationWidget();
    void stopUpdateRobotsThread();
    void startMapWorker();
    void stopMapWorker();
    void cancelRobotModifications();

private slots:
    void sendPathSelectedRobotSlot();
    void updateRobot(const QString ipAddress, const float posX, const float posY, const float ori);
    void updateMetadata(const int width, const int height, const float resolution, const float originX, const float originY);
    void updateMap(const QByteArray mapArray);
    void connectToRobot(bool checked);
    void quit(void);
    void setSelectedRobot(QPointer<RobotView> robotView);
    void setSelectedRobot(QAbstractButton* button);
    void setSelectedRobotNoParent(QAbstractButton *button);
    void setSelectedRobotFromPointSlot(QString robotName);
    void robotBtnEvent(void);
    void pointBtnEvent(void);
    void mapBtnEvent(void);
    void pathBtnEvent(void);
    void plusGroupBtnEvent(void);
    void minusGroupBtnEvent(void);
    void editGroupBtnEvent(void);
    void openLeftMenu(void);
    void backRobotBtnEvent(void);
    void editRobotBtnEvent(void);
    void checkRobotBtnEventMenu();
    void checkRobotBtnEvent(QString name);


    //void setCheckedRobot(QString name);
    void cancelEditSelecRobotBtnEvent(void);
    void robotSavedEvent(void);
    void saveRobotModifications(void);
    void minusSelecPointBtnEvent(void);
    void editSelecPointBtnEvent(void);
    void setSelectedPoint(void);
    void pointSavedEvent(QString groupName, double x, double y, QString name);
    void deletePath(int robotNb);
    void playSelectedRobot(int robotNb);
    void askForDeleteGroupConfirmation(const QString group);
    void askForDeletePointConfirmation(const QString index);
    void displayPointEvent(QString name, double x, double y);
    void askForDeleteDefaultGroupPointConfirmation(const QString index);
    void displayGroupMapEvent(void);
    void savePathSlot(void);
    void addPointPathSlot(QString name, double x, double y, GraphicItemState);
    void displayPointsInGroup(void);
    void removePointFromInformationMenu(void);
    void displayPointMapEvent(void);
    void editPointButtonEvent(void);
    void editTmpPathPointSlot(int id, QString name, double x, double y);
    void editPointFromGroupMenu(void);
    void saveEditPathPointSlot(void);
    void cancelEditPathPointSlot(void);
    void moveEditedPathPointSlot(void);
    void displayPointInfoFromGroupMenu(void);
    void updatePoint(void);
    void updateCoordinates(double x, double y);
    void removePointFromGroupMenu(void);
    void displayPointFromGroupMenu(void);
    void doubleClickOnPoint(QString checkedId);
    void doubleClickOnGroup(QString checkedId);
    void doubleClickOnPathsGroup(QString checkedButton);
    void reestablishConnectionsGroups();
    void reestablishConnectionsPoints();
    void createGroup(QString name);
    void modifyGroupWithEnter(QString name);
    void modifyGroupAfterClick(QString name);
    void enableReturnAndCloseButtons(void);
    void doubleClickOnRobot(QString checkedId);
    void setMessageCreationPath(QString message);
    void updateEditedPathPoint(double x, double y);
    void centerMap(void);
    void setMessageCreationPoint(QString type, CreatePointWidget::Error error);
    void choosePointName(QString message);
    void saveMapState(void);

    /**
     * @brief cancelEvent
     * Called when a user doesn't want to keep the modifications he's made on a point
     */
    void cancelEvent(void);
    void setMessageTop(const QString msgType, const QString msg);
    void setLastMessage(void) { setMessageTop(topLayout->getLastMessage().first, topLayout->getLastMessage().second); }
    void setMessageCreationGroup(QString type, QString message);
    void viewPathSelectedRobot(int robotNb, bool checked);
    void closeSlot();
    void setGraphicItemsState(const GraphicItemState state);
    void showHome();
    void showEditHome();
    void showAllHomes(void);
    void backEvent();
    void updateView();
    void robotIsAliveSlot(QString hostname, QString ip, QString mapId, QString ssid, int stage);
    void robotIsDeadSlot(QString hostname, QString ip);
    void selectViewRobot();
    void sendNewMapToRobots(QString ipAddress);
    void settingBtnSlot();
    void updatePathPainterPointViewSlot();
    void stopPath(int robotNb);
    void resetPathPointViewsSlot();
    void setEnableAll(bool enable, GraphicItemState state = GraphicItemState::NO_STATE, int noReturn = -1);
    void deletePathSelecRobotBtnEvent();
    void replacePoint(int id, QString name);
    void deletePathSlot(QString groupName, QString pathName);
    void editPathSlot(QString groupName, QString pathName);
    void displayPathSlot(QString groupName, QString pathName, bool display);
    void setNewHome(QString homeName);
    void goHome();
    void goHome(int nbRobot);

    /// for menu paths
    void displayGroupPaths();
    void editGroupPaths();
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

    void updateHomes(QString robot_name, QString home_pos);

protected:
    void moveEvent(QMoveEvent* event);
    bool changeRobotName(QString name);
    void changeRobotWifi(QString ssid, QString password);
    void stopMapThread();

private:
    Ui::MainWindow* ui;
    QThread serverThread;
    QThread mapThread;
    ScanMapWorker* mapWorker;
    RobotServerWorker* robotServerWorker;
    QVBoxLayout* rightLayout;
    CustomQGraphicsView* graphicsView;
    QSharedPointer<Map> map;
    QSharedPointer<Robots> robots;
    QGraphicsScene* scene;
    QPointer<MapView> mapPixmapItem;
    QPointer<RobotView> selectedRobot;
    QPointer<RobotView> scanningRobot;
    QSharedPointer<PointView> selectedPoint;
    QSharedPointer<Points> points;
    QSharedPointer<PointView> editedPointView;
    QVector<QSharedPointer<PointView>> pointViewsToDisplay;
    PathPainter* pathPainter;

    TopLayout* topLayout;
    QMessageBox msgBox;

    QList<QPair<QPair<QWidget*, QString>, MainWindow::WidgetType>> lastWidgets;
    LeftMenuWidget* leftMenuWidget;
    PointsLeftWidget* pointsLeftWidget;
    RobotsLeftWidget* robotsLeftWidget;
    MapLeftWidget* mapLeftWidget;
    EditSelectedRobotWidget* editSelectedRobotWidget;
    SelectedPointWidget* selectedPointWidget;
    CreatePointWidget* createPointWidget;
    LeftMenu* leftMenu;
    BottomLayout* bottomLayout;
    PathCreationWidget* robotPathCreationWidget;
    PathCreationWidget* pathCreationWidget;

    QPair<QPointF, float> mapState;

    QSharedPointer<Paths> paths;

    CommandController* commandController;
    std::string mapFile;
};

#endif // MAINWINDOW_H
