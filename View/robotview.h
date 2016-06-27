#ifndef ROBOTVIEW_H
#define ROBOTVIEW_H

class Robot;
class QGraphicsSceneMouseEvent;
class MapView;

#include <memory>
#include "Model/position.h"
#include <QObject>
#include <QGraphicsPolygonItem>
#include "Model/graphicitemstate.h"

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
    RobotView(const std::shared_ptr<Robot> &_robot, QGraphicsItem *parent);
    RobotView(QGraphicsItem* parent);

    /// Getter
    std::shared_ptr<Robot> getRobot(void) { return robot; }

    /// Setters
    void setRobot(std::shared_ptr<Robot> const& _robot) { robot = _robot; }
    void setPosition(const Position _position);
    void setPosition(const float x, const float y);
    void setOrientation(const float ori);
    void setSelected(const bool _selected);
    void display(const bool show) ;
    void setState(const GraphicItemState _state) { state = _state; }

signals:
    /**
     * @brief setSelectedSignal
     * Signal emitted when a robot is clicked on/selected
     */
    void setSelectedSignal(RobotView*);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void hoverEnterEvent(QGraphicsSceneHoverEvent *event);
    void hoverLeaveEvent(QGraphicsSceneHoverEvent *event);

private:
    GraphicItemState state;
    bool selected;
    std::shared_ptr<Robot> robot;
    bool shown;
};

#endif // ROBOTVIEW_H
