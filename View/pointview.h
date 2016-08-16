#ifndef POINTVIEW_H
#define POINTVIEW_H

class QGraphicsSceneMouseEvent;
class QMouseEvent;
class QDropEvent;

#include "Model/point.h"
#include <QObject>
#include <QGraphicsPixmapItem>
#include <QSharedPointer>
#include "Model/graphicitemstate.h"

#define SCALE 0.3
#define PIXMAP_NORMAL ":/icons/cropped_coordinates"
#define PIXMAP_MID ":/icons/blue_coord"
#define PIXMAP_START ":/icons/green_coord"
#define PIXMAP_STOP ":/icons/red_coord"
#define PIXMAP_HOVER ":/icons/orange_coord"
#define PIXMAP_START_STOP ":/icons/green_red_coord"

#define PIXMAP_HOME_NORMAL ":/icons/cropped_home"
#define PIXMAP_HOME_MID ":/icons/blue_home"
#define PIXMAP_HOME_START ":/icons/green_home"
#define PIXMAP_HOME_STOP ":/icons/red_home"
#define PIXMAP_HOME_HOVER ":/icons/orange_home"
#define PIXMAP_HOME_START_STOP ":/icons/green_red_home"
#include <qdebug.h>

/**
 * @brief The PointView class
 * This class provides a graphic object to display that's associated with a Point object
 */
class PointView : public QObject, public QGraphicsPixmapItem {
    Q_OBJECT

public:
    PointView(QSharedPointer<Point> const& point, QGraphicsItem *parent = 0);

    enum PixmapType{NORMAL, MID, START, STOP, HOVER, START_STOP};

    void setState(const GraphicItemState _state) { state = _state; }//  qDebug()<< "change state here" << point->getPosition().getX() << point->getPosition().getY() << _state;}
    GraphicItemState getState(void) const { return state; }
    void setPos(const qreal x, const qreal y);
    void setAddedToPath(const bool _addedToPath) { addedToPath = _addedToPath; }
    void setLastPixmap(const QPixmap _lastPixmap) { lastPixmap = _lastPixmap; }
    void setPixmap(const PixmapType pixType);
    //void setPixmap(const QPixmap &pixmap);
    void setPoint(QSharedPointer<Point> const& _point) { point = _point; }
    QSharedPointer<Point> getPoint(void) const { return point; }
    void setWasShown(const bool _wasShown) { wasShown = _wasShown; }
    bool getWasShown() const { return wasShown; }
    PixmapType getType(void) const { return type; }
    void setType(const PixmapType _type) { type = _type; }
    PixmapType getLastType(void) const { return lastType; }
    void setLastType(const PixmapType _last) { lastType = _last; }
    QPixmap getLastPixmap(void) const { return lastPixmap; }
    void setOriginalPosition(const Position position) { originalPosition = position; }
    Position getOriginalPosition(void) const { return originalPosition; }
    void updatePos(void);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void hoverEnterEvent(QGraphicsSceneHoverEvent *event);
    void hoverLeaveEvent(QGraphicsSceneHoverEvent *event);
    void dropEvent(QDropEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);

signals:
    void pointRightClicked(QString);
    void pointLeftClicked(QString);
    void addPointPath(QString name, double x, double y);
    void homeEdited(QString);
    void moveEditedPathPoint();
    void editedPointPositionChanged(double, double);
    void pathPointChanged(double, double);
    void hoverEventSignal(PointView::PixmapType, QString);
    void updatePathPainterPointView();

private:
    QSharedPointer<Point> point;
    GraphicItemState state;
    bool addedToPath;
    QPixmap lastPixmap;
    bool wasShown;
    PixmapType type;
    PixmapType lastType;
    Position originalPosition;
};

#endif // POINTVIEW_H
