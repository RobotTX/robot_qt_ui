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
class TopLayout;


#include "Model/points.h"
#include "View/robotview.h"
#include <QMainWindow>
#include <QModelIndex>
#include "Model/graphicitemstate.h"

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
    void stopPathCreation();
    int openConfirmMessage(const QString);
    void openInterdictionOfPointRemovalMessage(const QString pointName, const QString robotName);
    void clearNewMap();
    void disableMenu();
    void enableMenu();
    void clearPath(int robotNb);

signals:
    void sendCommand(QString);
    void nameChanged(QString, QString);

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
    void pointSavedEvent();
    void selectHomeEvent();
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
    void pathSaved(bool execPath);
    void addPathPoint(Point* point);
    void addPathPoint(PointView* pointView);
    void displayPointsInGroup(void);
    void updatePathPointToPainter(QVector<Point>* pointVector);
    void removePointFromInformationMenu(void);
    void displayPointMapEvent(void);
    void hidePathCreationWidget(void);
    void pointInfoEvent(void);
    void editPointButtonEvent(bool checked);
    void editTmpPathPointSlot(int id, Point* point, int nbWidget);
    void editPointFromGroupMenu(void);
       // prob need a different event
    void saveTmpEditPathPointSlot(void);
    void moveTmpEditPathPointSlot(void);
    void displayPointInfoFromGroupMenu(void);
    void updatePoint(void);
    void updateCoordinates(double x, double y);
    void removePointFromGroupMenu(void);
    void displayPointFromGroupMenu();
    /**
     * @brief cancelEvent
     * Called when a user doesn't want to keep the modifications he's made on a point
     */
    void cancelEvent(void);
    void setMessageTop(QString msgType, QString msg);
    void homeSelected(PointView* pointView, bool temporary);
    void showSelectedRobotWidgetSlot(void);
    void hideSelectedRobotWidgetSlot(void);
    void goHomeBtnEvent();
    void viewPathSelectedRobot(int robotNb);
    void setGraphicItemsState(const GraphicItemState state, const bool clear = false);



private:
    Ui::MainWindow* ui;
    ScanMetadataThread* metadataThread;
    ScanRobotThread* robotThread;
    ScanMapThread* mapThread;
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
    Points points;
    PathPainter* pathPainter;
    PointView* editedPointView;
    TopLayout* topLayout;
    QVector<PointView*> pathPointViews;

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
