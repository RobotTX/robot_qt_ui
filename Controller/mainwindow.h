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
class PointsView;
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


//#define XML_PATH "/home/m-a/Documents/QtProject/gobot-software/points.xml"
//#define ROBOTS_NAME_PATH "/home/m-a/Documents/QtProject/gobot-software/robotsName.dat"
//#define MAP_PATH "/home/m-a/Documents/QtProject/gobot-software/realMap.dat"

#define XML_PATH "/home/joan/Qt/QtProjects/gobot-software/points.xml"
#define ROBOTS_NAME_PATH "/home/joan/Qt/QtProjects/gobot-software/robotsName.dat"
#define MAP_PATH "/home/joan/Qt/QtProjects/gobot-software/realMap.dat"

//#define XML_PATH "/Users/fannylarradet/Desktop/GTRobots/gobot-software/points.xml"
//#define ROBOTS_NAME_PATH "/Users/fannylarradet/Desktop/GTRobots/gobot-software/robotsName.dat"
//#define MAP_PATH "/Users/fannylarradet/Desktop/GTRobots/gobot-software/realMap.dat"

#define PI 3.14159265
#define PORT_ROBOT_UPDATE 6000

namespace Ui {

class MainWindow;
}

class MainWindow : public QMainWindow{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    enum WidgetType { MENU, GROUPS, GROUP, POINT, ROBOTS, ROBOT };

    std::shared_ptr<Points> getPoints(void) const { return points; }
    QList<QPair<QPair<QWidget*, QString>, MainWindow::WidgetType>> getLastWidgets() const { return lastWidgets; }

    void initializeMenu();
    void initializeRobots();
    void initializePoints();
    void initializeBottomPanel();
    void initializeLeftMenu();
    void hideAllWidgets();
    void stopPathCreation();
    int openConfirmMessage(const QString);
    void openInterdictionOfPointRemovalMessage(const QString pointName, const QString robotName);
    int openEmptyGroupMessage(const QString groupName);
    void clearNewMap();
    void clearPath(const int robotNb);
    void resetFocus();
    void switchFocus(QString name, QWidget* widget, WidgetType type);
    /// to sleep for ms milliseconds
    void delay(const int ms) const;
    void setEnableAll(bool enable, GraphicItemState state = GraphicItemState::NO_STATE, bool clearPath = false, int noReturn = -1);
    void clearAllPath();

signals:
    void sendCommand(QString);
    void nameChanged(QString, QString);
    void changeCmdThreadRobotName(QString);

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
    void setSelectedRobotFromPoint();
    void robotBtnEvent();
    void pointBtnEvent();
    void mapBtnEvent();
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
    void backMapBtnEvent();
    //void setCheckedRobot(QString name);
    void cancelEditSelecRobotBtnEvent();
    void robotSavedEvent();
    void minusSelecPointBtnEvent();
    void editSelecPointBtnEvent();
    void setSelectedPoint(PointView* pointView, bool isTemporary);
    void pointSavedEvent(int index, double x, double y, QString name);
    void selectHomeEvent();
    void stopSelectedRobot(int robotNb);
    void playSelectedRobot(int robotNb);
    void askForDeleteGroupConfirmation(const int group);
    void askForDeletePointConfirmation(const int index);
    void displayPointEvent(PointView* _pointView);
    void askForDeleteDefaultGroupPointConfirmation(const int groupIndex);
    void displayGroupMapEvent(void);
    void pathSaved(bool execPath);
    void addPathPoint(Point* point);
    void addPathPoint(PointView* pointView);
    void displayPointsInGroup(void);
    void updatePathPointToPainter(QVector<Point> &pointVector, bool save);
    void removePointFromInformationMenu(void);
    void displayPointMapEvent(void);
    void hidePathCreationWidget(void);
    void editPointButtonEvent();
    void editTmpPathPointSlot(int id, Point* point, int nbWidget);
    void editPointFromGroupMenu(void);
    void saveTmpEditPathPointSlot(void);
    void moveTmpEditPathPointSlot(void);
    void displayPointInfoFromGroupMenu(void);
    void updatePoint(void);
    void updateCoordinates(double x, double y);
    void removePointFromGroupMenu(void);
    void displayPointFromGroupMenu();
    void doubleClickOnPoint(int checkedId);
    void doubleClickOnGroup(int checkedId);
    void reestablishConnectionsGroups();
    void reestablishConnectionsPoints();
    void removePoint(std::shared_ptr<Point>& point, const Origin origin);
    void createGroup(QString name);
    void modifyGroupWithEnter(QString name);
    void modifyGroupAfterClick(QString name);
    void enableReturnAndCloseButtons(void);
    void doubleClickOnRobot(int checkedId);
    void setMessageCreationPath(QString message);
    void updatePathPoint(double x, double y, PointView* pointView = 0);
    void centerMap();
    void setMessageCreationPoint(QString type, CreatePointWidget::Error error);
    void updatePathPainterPoints(int start, int row);
    void addPathPointToMap(Point* point);
    void updatePathPermanentPoint(QString, QString);

    /**
     * @brief cancelEvent
     * Called when a user doesn't want to keep the modifications he's made on a point
     */
    void cancelEvent(void);
    void setMessageTop(const QString msgType, const QString msg);
    void setLastMessage(void) { setMessageTop(topLayout->getLastMessage().first, topLayout->getLastMessage().second); }
    void setMessageCreationGroup(QString type, QString message);
    void homeSelected(PointView* pointView, bool temporary);
    void homeEdited(PointView* pointView, bool temporary);
    void goHomeBtnEvent();
    void viewPathSelectedRobot(int robotNb, bool checked);
    void editHomeEvent();
    void closeSlot();
    void setGraphicItemsState(const GraphicItemState state, const bool clear = false);
    void showHome();
    void hideHome(void);
    void backEvent();
    void updateView();
    void robotIsAliveSlot(QString hostname, QString ip, QString mapId, QString ssid);
    void robotIsDeadSlot(QString hostname, QString ip);
    void selectViewRobot();
    void sendNewMapToRobots(QString ipAddress);
    void sendNewMapToRobot(std::shared_ptr<Robot> robot, QString mapId);
    void settingBtnSlot();

private:
    Ui::MainWindow* ui;
    ScanMapThread* mapThread;
    UpdateRobotsThread* updateRobotsThread;
    QVBoxLayout* rightLayout;
    CustomQGraphicsView* graphicsView;
    std::shared_ptr<Map> map;
    std::shared_ptr<Robots> robots;
    QGraphicsScene* scene;
    MapView* mapPixmapItem;
    RobotView* selectedRobot;
    RobotView* scanningRobot;
    PointsView* pointViews;
    PointView* selectedPoint;
    std::shared_ptr<Points> points;
    PathPainter* pathPainter;
    PointView* editedPointView;
    TopLayout* topLayout;
    QVector<PointView*> pathPointViews;
    //QList<QPair<QWidget*, QString>> lastWidgets;
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
    PathCreationWidget* pathCreationWidget;
    QMessageBox msgBox;
    std::vector<PointView*> pointViewsToDisplay;
};

#endif // MAINWINDOW_H
