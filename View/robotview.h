#ifndef ROBOTVIEW_H
#define ROBOTVIEW_H

class QGraphicsSceneMouseEvent;
class MapView;

#include <QGraphicsPixmapItem>
#include <QSharedPointer>
#include "Model/position.h"
#include <QObject>
#include "Model/graphicitemstate.h"
#include "Model/robot.h"

#define ROBOT_HEIGHT_LOW 9
#define ROBOT_HEIGHT_HIGH 12
#define ROBOT_WIDTH 6

/**
 * @brief The RobotView class
 * A view that display one robot on the map
 */
class RobotView: public QObject, public QGraphicsPixmapItem {
    Q_OBJECT

public:
    RobotView(Robot* _robot, QGraphicsItem *parent);
    RobotView(QGraphicsItem* parent);

    /// Getters
    Robot* getRobot(void) { return robot; }
    int getLastStage(void) const { return lastStage; }

    /// Setters
    void setRobot(Robot* const& _robot) { robot = _robot; }
    void setPosition(const Position _position);
    void setPosition(const float x, const float y);
    void setOrientation(const float ori);
    void setSelected(const bool _selected);
    void display(const bool show) ;
    void setState(const GraphicItemState _state) { state = _state; }
    void setLastStage(const int _stage) { lastStage = _stage; }

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
    Robot* robot;
    bool selected;
    GraphicItemState state;
    bool shown;
    MapView* mapView;
    int lastStage;
};

#endif // ROBOTVIEW_H
