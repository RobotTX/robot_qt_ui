#ifndef MAPVIEW_H
#define MAPVIEW_H

class Point;
class QMainWindow;
class QMouseEvent;
class QGraphicsSceneMouseEvent;
class Map;

#include "View/pointview.h"
#include <QObject>
#include <QGraphicsPixmapItem>
#include <QSharedPointer>
#include "Model/graphicitemstate.h"
#include "Model/points.h"
#include "mainwindow.h"
#include "Model/point.h"


/**
 * @brief The MapView class
 * The class which display the map
 */
class MapView: public QObject, public QGraphicsPixmapItem {
    Q_OBJECT

public:
    MapView (const QPixmap& pixmap, const QSize size, QSharedPointer<Map> _map, QMainWindow *_mainWindow);

    QSize getSize(void) const { return size; }
    int getWidth(void) const { return size.width(); }
    int getHeight(void) const { return size.height(); }
    GraphicItemState getState(void) const { return state; }
    QMainWindow* getMainWindow(void) const { return mainWindow; }

    void setState(const GraphicItemState _state) { state = _state; }
    void setPoints(QSharedPointer<Points> _points) { points = _points; }

signals:
    /// emitted when a user left clicks the map
    void leftClick();
    /// emitted when a user is constructing a path for a robot
    void addPathPoint(QString, double, double);
    /// emitted when a user is constructing a path without a robot
    void addNoRobotPathPoint(QString, double, double);
    /// emitted when a user is editing a permanent point
    void newCoordinates(double, double);
    /// emitted when a user is editing a path point
    void newCoordinatesPathPoint(double, double);
    /// emitted when a user clicks an unknown part of the map to show him a message saying that he cannot create a point there
    void newMessage(QString);
    /// emitted when a user is scanning a map and want to send the robot somewhere
    void newScanningGoal(double, double);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private:
    QPointF dragStartPosition;
    QSize size;
    QPixmap tmpPointPixmap;
    QSharedPointer<Points> points;
    GraphicItemState state;
    QMainWindow* mainWindow;
    QSharedPointer<Map> map;
    int idTmp;
};

#endif // MAPVIEW_H

