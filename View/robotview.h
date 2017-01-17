#ifndef ROBOTVIEW_H
#define ROBOTVIEW_H

class QGraphicsSceneMouseEvent;


#include <QGraphicsPixmapItem>
#include <QSharedPointer>
#include "Model/position.h"
#include <QObject>
#include "Model/graphicitemstate.h"
#include "Model/robot.h"
#include <QPointer>
#include "View/mapview.h"

#define ROBOT_HEIGHT_LOW 9
#define ROBOT_HEIGHT_HIGH 12
#define ROBOT_WIDTH 28

/**
 * @brief The RobotView class
 * A view that display one robot on the map
 */
class RobotView : public QObject, public QGraphicsPixmapItem {
    Q_OBJECT

public:
    RobotView(QPointer<Robot> _robot, QPointer<MapView> parent);
    RobotView(QPointer<MapView> parent);

    /// Getters
    QPointer<Robot> getRobot(void) { return robot; }
    int getLastStage(void) const { return lastStage; }
    QVector<QPointF> getObstacles(void) const { return obstacles; }

    /// Setters
    void setRobot(QPointer<Robot> const& _robot) { robot = _robot; }
    void setPosition(const Position _position);
    void setPosition(const float x, const float y);
    void setOrientation(const float ori);
    void setSelected(const bool _selected);
    void display(const bool show) ;
    void setState(const GraphicItemState _state) { state = _state; }
    void setLastStage(const int _stage) { lastStage = _stage; }
    void setObstacles(const QVector<QPointF> _obstacles);


signals:
    /**
     * @brief setSelectedSignal
     * Signal emitted when a robot is clicked on/selected
     */
    void setSelectedSignal(QPointer<RobotView>);
    void updateLaser();

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *);
    void hoverEnterEvent(QGraphicsSceneHoverEvent *);
    void hoverLeaveEvent(QGraphicsSceneHoverEvent *);

private:
    QPointer<Robot> robot;
    bool selected;
    GraphicItemState state;
    bool shown;
    QPointer<MapView> mapView;
    int lastStage;
    QVector<QPointF> obstacles;
};

#endif // ROBOTVIEW_H
