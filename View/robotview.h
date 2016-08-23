#ifndef ROBOTVIEW_H
#define ROBOTVIEW_H

class Robot;
class QGraphicsSceneMouseEvent;
class MapView;

#include <QSharedPointer>
#include "Model/position.h"
#include <QObject>
#include "Model/graphicitemstate.h"
#include <QGraphicsPolygonItem>

#define ROBOT_HEIGHT_LOW 9
#define ROBOT_HEIGHT_HIGH 12
#define ROBOT_WIDTH 6

/**
 * @brief The RobotView class
 * A view that display one robot on the map
 */
class RobotView: public QObject, public QGraphicsPolygonItem {
    Q_OBJECT

public:
    RobotView(const QSharedPointer<Robot> &_robot, QGraphicsItem *parent);
    RobotView(QGraphicsItem* parent);

    /// Getter
    QSharedPointer<Robot> getRobot(void) { return robot; }

    /// Setters
    void setRobot(QSharedPointer<Robot> const& _robot) { robot = _robot; }
    void setPosition(const Position _position);
    void setPosition(const float x, const float y);
    void setOrientation(const float ori);
    void setSelected(const bool _selected);
    void display(const bool show) ;
    void setState(const GraphicItemState _state) { state = _state; }
    void setLastStage(const int _stage) { lastStage = _stage; }
    int getLastStage(void) const { return lastStage; }

signals:
    /**
     * @brief setSelectedSignal
     * Signal emitted when a robot is clicked on/selected
     */
    void setSelectedSignal(RobotView*);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *);
    void hoverEnterEvent(QGraphicsSceneHoverEvent *);
    void hoverLeaveEvent(QGraphicsSceneHoverEvent *);

private:
    QSharedPointer<Robot> robot;
    bool selected;
    GraphicItemState state;
    bool shown;
    MapView* mapView;
    int lastStage;
};

#endif // ROBOTVIEW_H
