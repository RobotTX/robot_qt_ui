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
                qDebug() << "right click on point NO STATE" ;
                emit pointRightClicked(this);
            }
            if(event->button() == Qt::LeftButton){
                qDebug() << "left click on point NO STATE" ;
                emit pointLeftClicked(this);
            }
        } else if(state == GraphicItemState::CREATING_PATH){
            qDebug() << "Clicked on a point while creating a path";
            addedToPath = true;
            emit addPointPath(this);
        } else if(state == GraphicItemState::EDITING){
            qDebug() << "(EDITING) PointView moving from" << pos().x() << pos().y();
        }  else if(state == GraphicItemState::EDITING_PERM){
            qDebug() << "Editing permanently";

        } else if(state == GraphicItemState::SELECTING_HOME){
            emit homeSelected(this, false);
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
    } else if(state == GraphicItemState::EDITING_PERM){
        float x = pos().x() + pixmap().width()*SCALE/2;
        float y = pos().y() + pixmap().height()*SCALE;
        emit editedPointPositionChanged(x, y);
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
    }else if(state == GraphicItemState::EDITING_PERM){
        QGraphicsPixmapItem::mouseReleaseEvent(event);
    }
}

void PointView::hoverEnterEvent(QGraphicsSceneHoverEvent *event){
    setToolTip(point->getName());
    setPixmap(PointView::PixmapType::MID);
}

void PointView::hoverLeaveEvent(QGraphicsSceneHoverEvent *event){
    QGraphicsPixmapItem::setPixmap(lastPixmap);
}

void PointView::setPos(const qreal x, const qreal y){
    QGraphicsPixmapItem::setPos(x - pixmap().width()*SCALE/2,
           y - pixmap().height()*SCALE);
}

/*void PointView::setPixmap(const QPixmap &pixmap){
    lastPixmap = pixmap;
    QGraphicsPixmapItem::setPixmap(pixmap);
}*/

void PointView::setPixmap(const PixmapType pixType){
    lastPixmap = this->pixmap();

    QPixmap pixmap;
    if(point->isHome()){
        switch(pixType){
            case NORMAL:
                pixmap = QPixmap(PIXMAP_HOME_NORMAL);
            break;
            case MID:
                pixmap = QPixmap(PIXMAP_HOME_MID);
            break;
            case START:
                pixmap = QPixmap(PIXMAP_HOME_START);
            break;
            case STOP:
                pixmap = QPixmap(PIXMAP_HOME_STOP);
            break;
            case HOVER:
                pixmap = QPixmap(PIXMAP_HOME_HOVER);
            break;
            case START_STOP:
                pixmap = QPixmap(PIXMAP_HOME_START_STOP);
            break;
            default:
                pixmap = QPixmap(PIXMAP_HOME_NORMAL);
            break;
        }
    } else {
        switch(pixType){
            case NORMAL:
                pixmap = QPixmap(PIXMAP_NORMAL);
            break;
            case MID:
                pixmap = QPixmap(PIXMAP_MID);
            break;
            case START:
                pixmap = QPixmap(PIXMAP_START);
            break;
            case STOP:
                pixmap = QPixmap(PIXMAP_STOP);
            break;
            case HOVER:
                pixmap = QPixmap(PIXMAP_HOVER);
            break;
            case START_STOP:
                pixmap = QPixmap(PIXMAP_START_STOP);
            break;
            default:
                pixmap = QPixmap(PIXMAP_NORMAL);
            break;
        }
    }
    QGraphicsPixmapItem::setPixmap(pixmap);
}
