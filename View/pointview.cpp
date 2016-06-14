#include "pointview.h"
#include "Model/point.h"
#include <QGraphicsSceneMouseEvent>
#include <QDebug>
#include <QPixmap>
#include <QMouseEvent>
#include "View/mapview.h"


PointView::PointView(std::shared_ptr<Point> _point) :
  QGraphicsPixmapItem(QPixmap(PIXMAP_NORMAL)),
  state(GraphicItemState::NO_STATE){
    setScale(SCALE);
    point = _point;
    setAcceptedMouseButtons(Qt::RightButton | Qt::LeftButton);
    setAcceptHoverEvents(true);
    setPos(point->getPosition().getX(),
           point->getPosition().getY());
    setToolTip(point->getName());
    setShapeMode(QGraphicsPixmapItem::BoundingRectShape);

    addedToPath = false;
    movable = false;
    lastPixmap = QPixmap(PIXMAP_NORMAL);
}

void PointView::mousePressEvent(QGraphicsSceneMouseEvent *event){
    if(!movable){
        if(state == GraphicItemState::NO_STATE){
            if(event->button() == Qt::RightButton){
                qDebug() << "right click on point" ;
                emit pointRightClicked(this);
            }
            if(event->button() == Qt::LeftButton){
                qDebug() << "left click on point" ;
                emit pointLeftClicked(this);
            }
        } else if(state == GraphicItemState::CREATING_PATH){
            qDebug() << "Clicked on a point while creating a path";
            addedToPath = true;
            emit addPointPath(this);
        } else {
            qDebug() << "Clicked on a point while in an unknown state";
        }
    } else {
        qDebug() << "PointView moving from" << pos().x() << pos().y();
    }
    QGraphicsPixmapItem::mousePressEvent(event);
}

void PointView::mouseMoveEvent(QGraphicsSceneMouseEvent *event){
    /*bool outOfMap = false;
    if(pos().x() < 0){

    }
    pos().x() > parentItem()->boundingRect().width()
    qDebug() << parentItem()->boundingRect().width() << parentItem()->boundingRect().height();
    qDebug() << "still moving2" << pos().x() << pos().y();*/
    QGraphicsPixmapItem::mouseMoveEvent(event);
}

void PointView::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    qDebug() << "to" << pos().x() << pos().y();
    QGraphicsPixmapItem::mouseReleaseEvent(event);
}

void PointView::hoverEnterEvent(QGraphicsSceneHoverEvent *event){
    QGraphicsPixmapItem::setPixmap(QPixmap(PIXMAP_HOVER));
}

void PointView::hoverLeaveEvent(QGraphicsSceneHoverEvent *event){
    setPixmap(lastPixmap);
}

void PointView::setPos(const qreal x, const qreal y){
    QGraphicsPixmapItem::setPos(x - pixmap().width()*SCALE/2,
           y - pixmap().height()*SCALE);
}

void PointView::setPixmap(const QPixmap &pixmap){
    lastPixmap = pixmap;
    QGraphicsPixmapItem::setPixmap(pixmap);
}
