#ifndef DRAWOBSTACLES_H
#define DRAWOBSTACLES_H

#include <QObject>
#include <QGraphicsItem>
#include <QGraphicsSceneMouseEvent>
#include "Model/robots.h"

class DrawObstacles : public QObject, public QGraphicsItem {
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)

public:
    explicit DrawObstacles(const QSize _size, QSharedPointer<Robots> _robots, QGraphicsItem *parent = Q_NULLPTR);

    QVector<QPointF> convertRangesToPoints(const float angle_min, const float angle_increment, const QVector<float> ranges, const QString ipAddress) const;

    void paint(QPainter *_painter, const QStyleOptionGraphicsItem *, QWidget *) Q_DECL_OVERRIDE;

protected:
    /**
     * @brief boundingRect
     * @return the size of the rectangle in which the items will be drawn
     * if the rectangle returned is too small the items might disappear
     * have to redefine this one correctly in order not to lose the items you draw (for example when you zoom)
     */
    QRectF boundingRect() const Q_DECL_OVERRIDE;

public:
    void removeRobotObstacles(const QString ipAddress);
    void clearRobotObstacles(const QString ipAddress);

private slots:
    void drawObstacles(float angle_min, float angle_max, float angle_increment, const QVector<float> &ranges, QString ipAddress);

    /// adds an entry to the map stored by the mapView in which the obstacles of the robot identified
    /// by the IP address are stored
    void addNewRobotObstacles(QString ipAddress);

private:
    QSize size;
    QSharedPointer<Robots> robots;
    /// keys are ip addresses of robots
    /// each robot has its own obstacles to draw represented by a vector of QPointF
    QMap<QString, QVector<QPointF>> obstacles;
};

#endif /// DRAWOBSTACLES_H
