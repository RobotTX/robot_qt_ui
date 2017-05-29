#ifndef OBSTACLESPAINTEDITEM_H
#define OBSTACLESPAINTEDITEM_H


#include <QtQuick/QQuickPaintedItem>

class ObstaclesPaintedItem : public QQuickPaintedItem {

    Q_OBJECT
    Q_PROPERTY(double orientation_ READ orientation WRITE setOrientation)
    Q_PROPERTY(double _x READ getX WRITE setX)
    Q_PROPERTY(double _y READ getY WRITE setY)

public:

    ObstaclesPaintedItem(QQuickItem* parent = 0);

    void paint(QPainter *painter) Q_DECL_OVERRIDE;

    double orientation() const { return orientation_; }
    double getX() const { return _x; }
    double getY() const { return _y; }

    void setOrientation(const double orientation) { orientation_ = orientation; }
    void setX(const double x) { _x = x; }
    void setY(const double y) { _y = y; }

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

private:
    QVector<QPointF> obstacles_;
    double orientation_;
    double _x;
    double _y;
    bool activated;
};

#endif /// OBSTACLESPAINTEDITEM_H
