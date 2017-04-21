#ifndef OBSTACLESPAINTEDITEM_H
#define OBSTACLESPAINTEDITEM_H


#include <QtQuick/QQuickPaintedItem>

class ObstaclesPaintedItem : public QQuickPaintedItem {

    Q_OBJECT
    Q_PROPERTY(float orientation_ READ orientation WRITE setOrientation)
    Q_PROPERTY(float _x READ getX WRITE setX)
    Q_PROPERTY(float _y READ getY WRITE setY)

public:

    ObstaclesPaintedItem(QQuickItem* parent = 0);

    void paint(QPainter *painter) Q_DECL_OVERRIDE;


    /**
     * @brief updateObstacles
     * @param angle_min
     * @param angle_max
     * @param angle_increment
     * @param ranges
     * To update the display of the obstacles on the map when we receive it from the robot
     */
    void updateObstacles(float angle_min, float angle_max, float angle_increment, QVector<float> ranges);

    /**
     * @brief clearObstacles
     * To remove all the obstacles when we stop using the laser
     */
    void clearObstacles(bool activated);

    float orientation() const { return orientation_; }
    float getX() const { return _x; }
    float getY() const { return _y; }

    void setOrientation(const float orientation) { orientation_ = orientation; }
    void setX(const float x) { _x = x; }
    void setY(const float y) { _y = y; }

private:
    QVector<QPointF> obstacles_;
    float orientation_;
    float _x;
    float _y;
    bool activated;
};

#endif /// OBSTACLESPAINTEDITEM_H
