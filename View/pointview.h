#ifndef POINTVIEW_H
#define POINTVIEW_H

class QGraphicsSceneMouseEvent;
class QMouseEvent;
class QDropEvent;
class RobotView;
class MainWindow;

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
    PointView(QSharedPointer<Point> const& point, MainWindow *_mainWindow);

    enum PixmapType { NORMAL, MID, START, STOP, HOVER, START_STOP, SELECTED };

    void setState(const GraphicItemState _state) { state = _state; }//  qDebug()<< "change state here" << point->getPosition().getX() << point->getPosition().getY() << _state;}
    void setPos(const qreal x, const qreal y);
    void setPixmap(const PixmapType pixType);
    void setPoint(QSharedPointer<Point> const& _point) { point = _point; }
    void setWasShown(const bool _wasShown) { wasShown = _wasShown; }
    PixmapType getType(void) const { return type; }
    void setType(const PixmapType _type) { type = _type; }
    void setLastType(const PixmapType _last) { lastType = _last; }
    void setOriginalPosition(const Position position) { originalPosition = position; }

    GraphicItemState getState(void) const { return state; }
    QSharedPointer<Point> getPoint(void) const { return point; }
    bool getWasShown() const { return wasShown; }
    PixmapType getLastType(void) const { return lastType; }
    Position getOriginalPosition(void) const { return originalPosition; }

    /**
     * @brief updatePos
     * to update the position of the pixmap when the position of the point changes
     */
    void updatePos(void);
    /**
     * @brief setToolTip
     * @param toolTip
     * sets a tooltip on the pointview
     */
    void setToolTip(const QString toolTip);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void hoverEnterEvent(QGraphicsSceneHoverEvent *event);
    void hoverLeaveEvent(QGraphicsSceneHoverEvent *event);
    void dropEvent(QDropEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);

signals:
    /// emitted when the point view catches a right click
    void pointRightClicked(QString);
    /// emitted when the point view catches a left click
    void pointLeftClicked(QString name, double x, double y);
    /// emitted when a path is being created so that a permanent point (corresponding to this point view) is to be added to the path
    void addPointPath(QString name, double x, double y, GraphicItemState);
    /// when an edited path point is dragged
    void moveEditedPathPoint();
    /// when an edited permanent point is dragged
    void editedPointPositionChanged(double, double);
    // might not be used anymore
    void editedHomePositionChanged(float, float, QString);
    /// emitted after a hover leave event to reset the path point view color
    void updatePathPainterPointView();
    /// emitted when a user is scanning a map and want to send the robot to this point
    void newScanningGoal(double, double);

private:
    QSharedPointer<Point> point;
    GraphicItemState state;
    bool wasShown;
    PixmapType type;
    PixmapType lastType;
    Position originalPosition;
    MainWindow* mainWindow;
};

#endif // POINTVIEW_H
