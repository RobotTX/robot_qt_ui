#ifndef MAINWINDOW_H
#define MAINWINDOW_H

class RobotView;
class PointView;
class ScanMapThread;
class UpdateRobotsThread;
class CustomQGraphicsView;
class Map;
class Robots;
class MapView;
class LeftMenuWidget;
class PointsLeftWidget;
class SelectedRobotWidget;
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

#include "Model/paths.h"
#include "View/createpointwidget.h"
#include "View/toplayout.h"
#include "Model/points.h"
#include "View/robotview.h"
#include <QMainWindow>
#include <QModelIndex>
#include "Model/graphicitemstate.h"
#include <QPair>
#include "Model/origin.h"
#include <QMessageBox>
#include "Model/point.h"
#include <QSettings>

/*
#define XML_PATH "/home/m-a/Documents/QtProject/gobot-software/points.xml"
#define ROBOTS_NAME_PATH "/home/m-a/Documents/QtProject/gobot-software/robotsName.dat"
#define MAP_PATH "/home/m-a/Documents/QtProject/gobot-software/realMap.dat"
#define PATHS_PATH "/home/m-a/Documents/QtProject/gobot-software/savedPaths.dat"
*/

#define XML_PATH "/home/joan/Qt/QtProjects/gobot-software/points.xml"
#define ROBOTS_NAME_PATH "/home/joan/Qt/QtProjects/gobot-software/robotsName.dat"
#define MAP_PATH "/home/joan/Qt/QtProjects/gobot-software/realMap.dat"
#define PATHS_PATH "/home/joan/Qt/QtProjects/gobot-software/savedPaths.dat"

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

    void initializeMenu();
    void initializeRobots();
    void initializePoints();
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
    void delay(const int ms) const;
    void setTemporaryMessageTop(const QString type, const QString message, const int ms);
    void updateAllPaths(void);
    void clearPath(const int robotNb);
    RobotView* getSelectedRobot(void) const { return selectedRobot; }
    void serializePaths(void);
    void deserializePaths(void);

signals:
    void sendCommand(QString);
    void nameChanged(QString, QString);
    void changeCmdThreadRobotName(QString);
    void addPathPoint(QString name, double x, double y);
    void addNoRobotPathPoint(QString name, double x, double y);
    void updatePathPainter();
    void updatePathPainterPointView();
    void resetPath();
    void resetPathCreationWidget();

private slots:
    void updateRobot(const QString ipAddress, const float posX, const float posY, const float ori);
    void updateMetadata(const int width, const int height, const float resolution, const float originX, const float originY);
    void updateMap(const QByteArray mapArray);
    void connectToRobot();
    void quit();
    void setSelectedRobot(RobotView* robotView);
    void editSelectedRobot(RobotView* robotView);
    void setSelectedRobot(QAbstractButton* button);
    void setSelectedRobotNoParent(QAbstractButton *button);
    void setSelectedRobotFromPointSlot(QString robotName);
    void robotBtnEvent();
    void pointBtnEvent();
    void mapBtnEvent();
    void pathBtnEvent();
    void plusGroupBtnEvent();
    void minusGroupBtnEvent();
    void editGroupBtnEvent();
    void selectPointBtnEvent();
    void openLeftMenu();
    void backSelecRobotBtnEvent();
    void editSelecRobotBtnEvent();
    void addPathSelecRobotBtnEvent();
    void backRobotBtnEvent();
    void editRobotBtnEvent();
    void checkRobotBtnEventMenu();
    void checkRobotBtnEventSelect();
    void checkRobotBtnEvent(QString name);
    void saveMapBtnEvent();
    void loadMapBtnEvent();
   // void backMapBtnEvent();
    //void setCheckedRobot(QString name);
    void cancelEditSelecRobotBtnEvent();
    void robotSavedEvent();
    void minusSelecPointBtnEvent();
    void editSelecPointBtnEvent();
    void setSelectedPoint();
    void pointSavedEvent(QString groupName, double x, double y, QString name);
    void deletePath(int robotNb);
    void playSelectedRobot(int robotNb);
    void askForDeleteGroupConfirmation(const QString group);
    void askForDeletePointConfirmation(const QString index);
    void displayPointEvent(QString name, double x, double y);
    void askForDeleteDefaultGroupPointConfirmation(const QString index);
    void displayGroupMapEvent(void);
    void savePathSlot();
    void cancelPathSlot();
    void addPointPathSlot(QString name, double x, double y);
    void displayPointsInGroup(void);
    void removePointFromInformationMenu(void);
    void displayPointMapEvent(void);
    void editPointButtonEvent();
    void editTmpPathPointSlot(int id, QString name, double x, double y);
    void editPointFromGroupMenu(void);
    void saveEditPathPointSlot(void);
    void cancelEditPathPointSlot(void);
    void moveEditedPathPointSlot(void);
    void displayPointInfoFromGroupMenu(void);
    void updatePoint(void);
    void updateCoordinates(double x, double y);
    void removePointFromGroupMenu(void);
    void displayPointFromGroupMenu();
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
    void centerMap();
    void setMessageCreationPoint(QString type, CreatePointWidget::Error error);
    void choosePointName(QString message);
    void saveMapState();

    /**
     * @brief cancelEvent
     * Called when a user doesn't want to keep the modifications he's made on a point
     */
    void cancelEvent(void);
    void setMessageTop(const QString msgType, const QString msg);
    void setLastMessage(void) { setMessageTop(topLayout->getLastMessage().first, topLayout->getLastMessage().second); }
    void setMessageCreationGroup(QString type, QString message);
    void goHomeBtnEvent();
    void viewPathSelectedRobot(int robotNb, bool checked);
    void editHomeEvent();
    void closeSlot();
    void setGraphicItemsState(const GraphicItemState state);
    void showHome();
    void showEditHome();
    void hideHome(void);
    void backEvent();
    void updateView();
    void robotIsAliveSlot(QString hostname, QString ip, QString mapId, QString ssid, int stage);
    void robotIsDeadSlot(QString hostname, QString ip);
    void selectViewRobot();
    void sendNewMapToRobots(QString ipAddress);
    void sendNewMapToRobot(QSharedPointer<Robot> robot, QString mapId);
    void settingBtnSlot();
    void updatePathPainterPointViewSlot();
    void stopPath(int robotNb);
    void resetPathPointViewsSlot();
    void setEnableAll(bool enable, GraphicItemState state = GraphicItemState::NO_STATE, int noReturn = -1);
    void deletePathSelecRobotBtnEvent();
    void updateHomeCoordinates(float x, float y, QString name);
    void replacePoint(int id, QString name);
    void deletePathSlot(QString groupName, QString pathName);
    void editPathSlot(QString groupName, QString pathName);
    void displayPathSlot(QString groupName, QString pathName, bool display);

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
    void addNoRobotPointPathSlot(QString name, double x, double y);
    void cancelEditNoRobotPathPointSlot();
    void cancelNoRobotPathSlot();
    void saveNoRobotPathSlot();

private:
    Ui::MainWindow* ui;
    ScanMapThread* mapThread;
    UpdateRobotsThread* updateRobotsThread;
    QVBoxLayout* rightLayout;
    CustomQGraphicsView* graphicsView;
    QSharedPointer<Map> map;
    QSharedPointer<Robots> robots;
    QGraphicsScene* scene;
    MapView* mapPixmapItem;
    RobotView* selectedRobot;
    RobotView* scanningRobot;
    QSharedPointer<PointView> selectedPoint;
    QSharedPointer<Points> points;
    QSharedPointer<PointView> editedPointView;
    QVector<QSharedPointer<PointView>> pointViewsToDisplay;
    PathPainter* robotPathPainter;
    PathPainter* noRobotPathPainter;

    TopLayout* topLayout;
    QMessageBox msgBox;

    QList<QPair<QPair<QWidget*, QString>, MainWindow::WidgetType>> lastWidgets;
    LeftMenuWidget* leftMenuWidget;
    PointsLeftWidget* pointsLeftWidget;
    SelectedRobotWidget* selectedRobotWidget;
    RobotsLeftWidget* robotsLeftWidget;
    MapLeftWidget* mapLeftWidget;
    EditSelectedRobotWidget* editSelectedRobotWidget;
    SelectedPointWidget* selectedPointWidget;
    CreatePointWidget* createPointWidget;
    LeftMenu* leftMenu;
    BottomLayout* bottomLayout;
    PathCreationWidget* robotPathCreationWidget;
    PathCreationWidget* noRobotPathCreationWidget;

    QPair<QPointF, float> mapState;

    QSharedPointer<Paths> paths;
    QSettings settings;
};

#endif // MAINWINDOW_H
