#ifndef MAPVIEW_H
#define MAPVIEW_H

class PointView;
class PointsView;
class Point;
class QMainWindow;
class QMouseEvent;
class QGraphicsSceneMouseEvent;

#include <QObject>
#include <QGraphicsPixmapItem>
#include <QSharedPointer>
#include "Model/enumgraphicstate.h"
#include "Model/points.h"

/**
 * @brief The MapView class
 * The class which display the map
 */
class MapView: public QObject, public QGraphicsPixmapItem {
    Q_OBJECT

public:
    MapView (const QPixmap& pixmap, const QSize size, PointsView* const& points, QMainWindow *_mainWindow);
    ~MapView();

    /// Getters
    QSize getSize(void) const { return size; }
    int getWidth(void) const { return size.width(); }
    int getHeight(void) const { return size.height(); }
    QVector<PointView*> getPathCreationPoints(void) const { return pathCreationPoints; }
    PointView* getTmpPointView(void) const { return tmpPointView; }

    /// Setter
    void setPoint(const QSharedPointer<PointView> _point) { point = _point; }
    void setState(const GraphicItemState _state);

public:
    void updatePoints(const Points& points);

signals:
    void pointLeftClicked(PointView*, bool);
    void addPathPointMapView(Point*);

private slots:
    void addPathPointMapViewSlot(PointView*);
    void updateHover(QString, QString);

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
};

#endif // MAPVIEW_H

