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
    MapView (const QPixmap& pixmap, const QSize size, std::shared_ptr<Map> _map, QMainWindow *_mainWindow);

    /// Getters
    QSize getSize(void) const { return size; }
    int getWidth(void) const { return size.width(); }
    int getHeight(void) const { return size.height(); }
    GraphicItemState getState(void) const { return state; }
    QMainWindow* getMainWindow(void) const { return mainWindow; }

    /// Setters
    void setState(const GraphicItemState _state);
    void setPoints(std::shared_ptr<Points> _points);

    void addPathPoint(PointView* pointView);
    void changeOrderPathPoints(const int start, const int row);
    void addPermanentPointToPath(PointView* point);
    void replacePermanentPathPoint(const int index, PointView *const pv);
    int findIndexInPathByName(const QString name);

signals:
    void pointLeftClicked(QString);
    void addPathPointMapView(Point*);
    void homeSelected(QString pointView, bool temporary);
    void homeEdited(QString pointView, bool temporary);
    void newCoordinates(double x, double y);
    void newCoordinatesPathPoint(double, double);
    void newMessage(QString);

private slots:
    void addPathPointMapViewSlot(PointView*);
    void updateHover(QString, QString);
    void updatePixmapHover(PointView::PixmapType type, QString pv);
    void addPointEditPath(Point pt);
    void deletePointView(Point pt);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private:
    QPointF dragStartPosition;
    QSize size;
    QPixmap tmpPointPixmap;
    std::shared_ptr<Points> points;
    GraphicItemState state;
    QMainWindow* mainWindow;
    std::shared_ptr<Map> map;
    int idTmp;
};

#endif // MAPVIEW_H

