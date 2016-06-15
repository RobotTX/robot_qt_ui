#ifndef MAINWINDOW_H
#define MAINWINDOW_H

class RobotView;
class PointView;
class ScanMetadataThread;
class ScanRobotThread;
class ScanMapThread;
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
class EditSelectedPointWidget;
class LeftMenu;
class BottomLayout;
class PathCreationWidget;
class QAbstractButton;
class QVBoxLayout;
class PathPainter;

#include "Model/points.h"
#include "View/robotview.h"
#include <QMainWindow>
#include <QModelIndex>
#include "Model/enumgraphicstate.h"

#define PI 3.14159265
#define PORT_MAP_METADATA 4000
#define PORT_ROBOT_POS 4001
#define PORT_MAP 4002
#define PORT_CMD 5600

namespace Ui {

class MainWindow;
}

class MainWindow : public QMainWindow{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();


    Points getPoints(void) const { return points; }

    void initializeMenu();
    void initializeRobots();
    void initializePoints();
    void initializeBottomPanel();
    void initializeLeftMenu();
    void hideAllWidgets();
    void setGraphicItemsState(GraphicItemState state);
    void stopPathCreation();

signals:
    void sendCommand(QString);

private slots:
    void updateRobot(const float posX, const float posY, const float ori);
    void updateMetadata(const int width, const int height, const float resolution
                        , const float originX, const float originY);
    void updateMap(const QByteArray mapArray);
    void connectToRobot();
    void quit();
    void setSelectedRobot(RobotView* robotView);
    void editSelectedRobot(RobotView* robotView);
    void setSelectedRobot(QAbstractButton* button);
    void robotBtnEvent();
    void pointBtnEvent();
    void mapBtnEvent();
    void backGroupBtnEvent();
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
    void checkRobotBtnEvent();
    void saveMapBtnEvent();
    void loadMapBtnEvent();
    void backMapBtnEvent();
    void setCheckedRobot(QAbstractButton* button, bool checked);
    void cancelEditSelecRobotBtnEvent();
    void robotSavedEvent();
    void backSelecPointBtnEvent();
    void minusSelecPointBtnEvent();
    void editSelecPointBtnEvent();
    void setSelectedPoint(PointView* pointView, bool isTemporary);
    //void cancelEditSelecPointBtnEvent();
    void pointSavedEvent();
    void selectHomeEvent();
    void showHomeEvent();
    void displayDeleteEvent(QModelIndex index);
    void backToGroupsButtonEvent(void);
    void stopSelectedRobot(int robotNb);
    void playSelectedRobot(int robotNb);
    void askForDeleteGroupConfirmation(const int group);
    void askForDeletePointConfirmation(const int index);
    void displayPointEvent(PointView* _pointView);
    void modifyGroupEvent(const int groupIndex);
    /// executed when an item of the list is clicked
    void displayGroupEvent(int groupIndex, bool display);
    /// executed when the map button is clicked
    void askForDeleteDefaultGroupPointConfirmation(const int groupIndex);
    void removeGroupEvent(const int groupIndex);
    void backPathCreation(void);
    void displayGroupMapEvent(void);
    void pathSaved(void);
    void addPathPoint(Point* point);
    void addPathPoint(PointView* pointView);
    void displayPointsInGroup(void);
    void updatePathPointToPainter(QVector<Point>* pointVector);
    void removePointFromInformationMenu(void);
    void displayPointMapEvent(void);
    void hidePathCreationWidget(void);
    void pointInfoEvent(void);
    void editPointButtonEvent(void);
    void editTmpPathPointSlot(int id, Point* point, int nbWidget);
    void editPointFromGroupMenu(void);
       // prob need a different event

private:
    Ui::MainWindow* ui;
    ScanMetadataThread* metadataThread;
    ScanRobotThread* robotThread;
    ScanMapThread* mapThread;
    QVBoxLayout* rightLayout;
    QToolBar* toolbar;
    CustomQGraphicsView* graphicsView;
    Map* map;
    Robots* robots;
    QGraphicsScene* scene;
    MapView* mapPixmapItem;
    QMenuBar* menuBar;
    QAction* addPointAction;
    RobotView* selectedRobot;
    RobotView* scanningRobot;
    PointsView* pointViews;
    PointView* selectedPoint;
    Points points;
    PathPainter* pathPainter;

    QWidget* lastWidget;
    LeftMenuWidget* leftMenuWidget;
    PointsLeftWidget* pointsLeftWidget;
    SelectedRobotWidget* selectedRobotWidget;
    RobotsLeftWidget* robotsLeftWidget;
    MapLeftWidget* mapLeftWidget;
    EditSelectedRobotWidget* editSelectedRobotWidget;
    SelectedPointWidget* selectedPointWidget;
    EditSelectedPointWidget* editSelectedPointWidget;
    LeftMenu* leftMenu;
    BottomLayout* bottomLayout;
    PathCreationWidget* pathCreationWidget;
};

#endif // MAINWINDOW_H
