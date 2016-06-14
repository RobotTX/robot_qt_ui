#ifndef POINTVIEW_H
#define POINTVIEW_H

class Point;
class QGraphicsSceneMouseEvent;
class QMouseEvent;
class QDropEvent;

#include <QObject>
#include <QGraphicsPixmapItem>
#include <memory>
#include "Model/enumgraphicstate.h"

#define SCALE 0.3
#define PIXMAP_NORMAL ":/icons/cropped_coordinates"
#define PIXMAP_MID ":/icons/blue_coord"
#define PIXMAP_START ":/icons/green_coord"
#define PIXMAP_STOP ":/icons/red_coord"
#define PIXMAP_HOVER ":/icons/orange_coord"

/**
 * @brief The PointView class
 * This class provides a graphic object to display that's associated with a Point object
 */
class PointView : public QObject, public QGraphicsPixmapItem {
    Q_OBJECT

public:
    PointView(std::shared_ptr<Point> point);

    std::shared_ptr<Point> getPoint(void) const { return point; }

    void setState(const GraphicItemState _state) { state = _state; }
    void setPos(const qreal x, const qreal y);
    void setAddedToPath(const bool _addedToPath) { addedToPath = _addedToPath; }
    void setLastPixmap(const QPixmap& _lastPixmap) { lastPixmap = _lastPixmap; }
    void setPixmap(const QPixmap& pixmap);

    void setMovable(const bool _movable) { movable = _movable; }

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void hoverEnterEvent(QGraphicsSceneHoverEvent *event);
    void hoverLeaveEvent(QGraphicsSceneHoverEvent *event);
    void dropEvent(QDropEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);

signals:
    void pointRightClicked(PointView*);
    void pointLeftClicked(PointView*);
    void addPointPath(PointView*);

private:
    std::shared_ptr<Point> point;
    GraphicItemState state;
    bool addedToPath;
    bool movable;
    QPixmap lastPixmap;
};

#endif // POINTVIEW_H
