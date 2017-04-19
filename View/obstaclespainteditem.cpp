#include "obstaclespainteditem.h"
#include <QPainter>

ObstaclesPaintedItem::ObstaclesPaintedItem(QQuickItem *parent) : QQuickPaintedItem(parent) {
    setFlag(QQuickItem::ItemHasContents, true);
}

void ObstaclesPaintedItem::paint(QPainter *painter){
    painter->setPen(Qt::red);
    for(int j = 0; j < obstacles_.size(); j++)
        painter->drawPoint(obstacles_.at(j).x() + 300, obstacles_.at(j).y() + 300);
}

void ObstaclesPaintedItem::setObstacles(const QVector<QPointF>& obs){
    //qDebug() << "ObstaclesPaintedItem::setObstacles called";
    obstacles_ = obs;
    update();
}
