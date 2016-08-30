#ifndef POINTVIEW_H
#define POINTVIEW_H

class QGraphicsSceneMouseEvent;
class QMouseEvent;
class QDropEvent;
class RobotView;

#include "Model/point.h"
#include <QObject>
#include <QGraphicsPixmapItem>
#include <QSharedPointer>
#include "Model/graphicitemstate.h"

#define SCALE 0.3
#define PIXMAP_NORMAL ":/icons/coordinates.png"
#define PIXMAP_MID ":/icons/blue_coord.png"
#define PIXMAP_SELECTED ":/icons/orange_coord.png"
#define PIXMAP_START ":/icons/green_coord.png"
#define PIXMAP_STOP ":/icons/red_coord.png"
#define PIXMAP_HOVER ":/icons/violet_coord.png"
#define PIXMAP_START_STOP ":/icons/green_red_coord.png"

#define PIXMAP_HOME_NORMAL ":/icons/home.png"
#define PIXMAP_HOME_MID ":/icons/blue_home.png"
#define PIXMAP_HOME_SELECTED ":/icons/orange_home.png"
#define PIXMAP_HOME_START ":/icons/green_home.png"
#define PIXMAP_HOME_STOP ":/icons/red_home.png"
#define PIXMAP_HOME_HOVER ":/icons/violet_home.png"
#define PIXMAP_HOME_START_STOP ":/icons/green_red_home.png"

/**
 * @brief The PointView class
 * This class provides a graphic object to display that's associated with a Point object
 */
class PointView : public QObject, public QGraphicsPixmapItem {
    Q_OBJECT

public:
    PointView(QSharedPointer<Point> const& point, QGraphicsItem *parent);

    enum PixmapType{NORMAL, MID, START, STOP, HOVER, START_STOP, SELECTED};

    void setState(const GraphicItemState _state) { state = _state; }//  qDebug()<< "change state here" << point->getPosition().getX() << point->getPosition().getY() << _state;}
    GraphicItemState getState(void) const { return state; }
    void setPos(const qreal x, const qreal y);
    void setAddedToPath(const bool _addedToPath) { addedToPath = _addedToPath; }
    void setPixmap(const PixmapType pixType, RobotView *selectedRobot = 0);
    //void setPixmap(const QPixmap &pixmap);
    void setPoint(QSharedPointer<Point> const& _point) { point = _point; }
    QSharedPointer<Point> getPoint(void) const { return point; }
    void setWasShown(const bool _wasShown) { wasShown = _wasShown; }
    bool getWasShown() const { return wasShown; }
    PixmapType getType(void) const { return type; }
    void setType(const PixmapType _type) { type = _type; }
    PixmapType getLastType(void) const { return lastType; }
    void setLastType(const PixmapType _last) { lastType = _last; }
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
    void pointLeftClicked(QString name, double x, double y);
    void addPointPath(QString name, double x, double y);
    void addNoRobotPointPath(QString name, double x, double y);
    void homeEdited(QString);
    void moveEditedPathPoint();
    void editedPointPositionChanged(double, double);
    void editedHomePositionChanged(float, float, QString);
    void pathPointChanged(double, double);
    void hoverEventSignal(PointView::PixmapType, QString);
    void updatePathPainterPointView();

private:
    QSharedPointer<Point> point;
    GraphicItemState state;
    bool addedToPath;
    bool wasShown;
    PixmapType type;
    PixmapType lastType;
    Position originalPosition;
    RobotView* selectedRobot;
};

#endif // POINTVIEW_H
