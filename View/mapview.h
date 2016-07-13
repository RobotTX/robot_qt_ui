#ifndef MAPVIEW_H
#define MAPVIEW_H

class PointsView;
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


/**
 * @brief The MapView class
 * The class which display the map
 */
class MapView: public QObject, public QGraphicsPixmapItem {
    Q_OBJECT

public:
    MapView (const QPixmap& pixmap, const QSize size, std::shared_ptr<Map> _map, QMainWindow *_mainWindow);
    ~MapView();

    /// Getters
    QSize getSize(void) const { return size; }
    int getWidth(void) const { return size.width(); }
    int getHeight(void) const { return size.height(); }
    QVector<PointView*> getPathCreationPoints(void) const { return pathCreationPoints; }
    PointView* getTmpPointView(void) const { return tmpPointView; }
    GraphicItemState getState(void) const { return state; }
    PointsView* getPermanentPoints(void) const { return permanentPoints; }
    QMainWindow* getMainWindow(void) const { return mainWindow; }

    /// Setters
    void setPoint(const QSharedPointer<PointView> _point) { point = _point; }
    void setState(const GraphicItemState _state, const bool clear = false);
    void setPermanentPoints(const std::shared_ptr<Points> &points);
    void setPermanentPoints(PointsView* pointsView);

    void addPathPoint(PointView* pointView);
    void clearPointViews();
    void addPointView(PointView * const &_pointView);
    void updatePoints(const Points& points);

signals:
    void pointLeftClicked(PointView*, bool);
    void addPathPointMapView(Point*);
    void homeSelected(PointView* pointView, bool temporary);
    void homeEdited(PointView* pointView, bool temporary);
    void newCoordinates(double x, double y);
    void newCoordinatesPathPoint(double, double);
    void newMessage(QString);

private slots:
    void addPathPointMapViewSlot(PointView*);
    void updateHover(QString, QString);
    void updatePixmapHover(PointView::PixmapType type, PointView* pv);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private:
    QPointF dragStartPosition;
    QSharedPointer<PointView> point;
    QSize size;
    QPixmap tmpPointPixmap;
    PointsView* permanentPoints;
    GraphicItemState state;
    QMainWindow* mainWindow;
    QVector<PointView*> pathCreationPoints;
    PointView* tmpPointView;
    std::shared_ptr<Map> map;
};

#endif // MAPVIEW_H

