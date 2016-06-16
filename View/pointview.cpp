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

    addedToPath = false;
    setShapeMode(QGraphicsPixmapItem::BoundingRectShape);

    addedToPath = false;
    lastPixmap = QPixmap(PIXMAP_NORMAL);
}

void PointView::mousePressEvent(QGraphicsSceneMouseEvent *event){
        if(state == GraphicItemState::NO_STATE){
            if(event->button() == Qt::RightButton){
                qDebug() << "right click on point" ;
                emit pointRightClicked(this);
            }
            if(event->button() == Qt::LeftButton){
                qDebug() << "left click on point" ;
                emit pointLeftClicked(this);
                emit superimposePointView();
            }
        } else if(state == GraphicItemState::CREATING_PATH){
            qDebug() << "Clicked on a point while creating a path";
            addedToPath = true;
            emit addPointPath(this);
        } else if(state == GraphicItemState::EDITING){
            qDebug() << "PointView moving from" << pos().x() << pos().y();
        } else {
            qDebug() << "(PointView " << point->getName() << ") NO EVENT";
        }
}

void PointView::mouseMoveEvent(QGraphicsSceneMouseEvent *event){
    if(state == GraphicItemState::EDITING){
        /*bool outOfMap = false;
        if(pos().x() < 0){

        }
        pos().x() > parentItem()->boundingRect().width()
        qDebug() << parentItem()->boundingRect().width() << parentItem()->boundingRect().height();
        qDebug() << "still moving" << pos().x() << pos().y();*/

        float x = pos().x() + pixmap().width()*SCALE/2;
        float y = pos().y() + pixmap().height()*SCALE;
        point->setPosition(x, y);
        emit moveTmpEditPathPoint();
        QGraphicsPixmapItem::mouseMoveEvent(event);
    }
}

void PointView::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    if(state == GraphicItemState::EDITING){
        float x = pos().x() + pixmap().width()*SCALE/2;
        float y = pos().y() + pixmap().height()*SCALE;
        qDebug() << "to" << x << y;
        point->setPosition(x, y);
        QGraphicsPixmapItem::mouseReleaseEvent(event);
    }
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
