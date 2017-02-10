#include "drawobstacles.h"
#include <QDebug>
#include <QPainter>
#include <assert.h>
#include "View/Map/mapview.h"
#include "Model/Robots/robots.h"
#include "Controller/mainwindow.h"
#include "Model/Robots/robot.h"
#include <QtMath>

DrawObstacles::DrawObstacles(const QSize _size, QSharedPointer<Robots> _robots, QGraphicsItem *parent)
    : QGraphicsItem(parent), size(_size), robots(_robots){
}

/// the event that is called when calling update() and which is drawing on the map
void DrawObstacles::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *){
    painter->setPen(Qt::red);

    for(int i = 0; i < robots->getRobotsVector().size(); i++){
        QPointer<RobotView> robotView = robots->getRobotsVector().at(i);
        if(robotView){
            QVector<QPointF> obstacles = robotView->getObstacles();
            for(int j = 0; j < obstacles.size(); j++)
                painter->drawPoint(QPointF(robotView->getRobot()->getPosition().getX() + obstacles.at(j).x(),
                                           robotView->getRobot()->getPosition().getY() + obstacles.at(j).y()));
        }
    }
}

QRectF DrawObstacles::boundingRect() const {
    /// that's so that when you zoom your points don't disappear
    return QRectF(0, 0, 5 * size.width(), 5 * size.height());
}
