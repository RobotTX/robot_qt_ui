#ifndef MAPVIEW_H
#define MAPVIEW_H

class Point;
class QMainWindow;
class QMouseEvent;
class QGraphicsSceneMouseEvent;
class Map;
class DrawObstacles;
class PointView;

#include "View/Points/pointview.h"
#include <QObject>
#include <QGraphicsPixmapItem>
#include <QSharedPointer>
#include "Model/Other/graphicitemstate.h"
#include "Model/Points/points.h"
#include "Controller/mainwindow.h"
#include "Model/Points/point.h"
#include <QPainter>

/**
 * @brief The MapView class
 * The class which display the map
 */
class MapView: public QObject, public QGraphicsPixmapItem {
    Q_OBJECT

public:
    MapView (const QPixmap& pixmap, const QSize size);

    QSize getSize(void) const { return size; }
    int getWidth(void) const { return size.width(); }
    int getHeight(void) const { return size.height(); }
    GraphicItemState getState(void) const { return state; }

    void setState(const GraphicItemState _state) { state = _state; }
    void setTmpPointView(QSharedPointer<PointView> pv) { tmpPointView = pv; }


signals:
    /// emitted when a user left clicks the map
    void leftClick();
    /// emitted when a user is constructing a path for a robot
    void addPathPoint(QString, double, double);
    /// emitted when a user is editing a permanent point
    void newCoordinates(double, double);
    /// emitted when a user is editing a path point
    void newCoordinatesPathPoint(double, double);
    /// emitted when a user clicks an unknown part of the map to show him a message saying that he cannot create a point there
    void newMessage(QString);
    void testCoord(double, double);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private:
    QSize size;
    QSharedPointer<PointView> tmpPointView;
    QPointF dragStartPosition;
    QPixmap tmpPointPixmap;
    GraphicItemState state;
};

#endif /// MAPVIEW_H

