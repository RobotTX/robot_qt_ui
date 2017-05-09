#include "obstaclespainteditem.h"
#include <QPainter>
#include <QtMath>

ObstaclesPaintedItem::ObstaclesPaintedItem(QQuickItem *parent) : QQuickPaintedItem(parent), activated(false) {
    setFlag(QQuickItem::ItemHasContents, true);
}

void ObstaclesPaintedItem::paint(QPainter *painter){
    painter->setPen(Qt::red);
    for(int j = 0; j < obstacles_.size(); j++)
        painter->drawPoint(obstacles_.at(j).x() + 300, obstacles_.at(j).y() + 300);
}

void ObstaclesPaintedItem::updateObstacles(double angle_min, double angle_max, double angle_increment, QVector<double> ranges){
    if(activated){
        setPosition(QPointF(_x, _y));
        obstacles_.clear();
        int i(ranges.size()-1);
        /// for improved performance
        std::for_each(ranges.begin(), ranges.end(), [&](const double range) {
            /// rotation is done on the qml side
            obstacles_.push_back(QPointF(range * qCos(orientation_*3.14159/180 - 3.14159/2 + angle_min + i*angle_increment) * 20 ,
                                     range * qSin(orientation_*3.14159/180 - 3.14159/2 + angle_min + i*angle_increment) * 20)); i--; });
        update();
    } //else
        //qDebug() << "ObstaclesPaintedItem::updateObstacles Got data while the laser was not activated";
}

void ObstaclesPaintedItem::clearObstacles(bool _activated){
    activated = _activated;
    obstacles_.clear();
    update();
}
