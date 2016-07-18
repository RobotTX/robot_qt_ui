#include "pointview.h"
#include "Model/point.h"
#include <QGraphicsSceneMouseEvent>
#include <QDebug>
#include <QPixmap>
#include <QMouseEvent>


PointView::PointView(std::shared_ptr<Point> _point, QGraphicsItem* parent) :
  QGraphicsPixmapItem(QPixmap(PIXMAP_NORMAL), parent), state(GraphicItemState::NO_STATE), type(PixmapType::NORMAL), lastType(PixmapType::NORMAL)
{
    setScale(SCALE);
    point = _point;
    setAcceptedMouseButtons(Qt::RightButton | Qt::LeftButton);
    setAcceptHoverEvents(true);
    setPos(point->getPosition().getX(),
           point->getPosition().getY());
    setToolTip(point->getName());

    addedToPath = false;
    wasShown = true;
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
            qDebug() << "Editing permanently PointView moving from" << pos().x() << pos().y();;
        } else if(state == GraphicItemState::SELECTING_HOME){
            emit homeSelected(this, false);
        } else if(state == GraphicItemState::EDITING_HOME){
            emit homeEdited(this, false);
        } else {
            qDebug() << "(PointView " << point->getName() << ") NO EVENT";
        }
}

void PointView::mouseMoveEvent(QGraphicsSceneMouseEvent *event){

    if(state == GraphicItemState::EDITING || state == GraphicItemState::EDITING_PERM){
        QGraphicsItem::mouseMoveEvent(event);

        float x = pos().x() + pixmap().width()*SCALE/2;
        float y = pos().y() + pixmap().height()*SCALE;

        if(x < 0){
            x = 0;
        }
        if(x > parentItem()->boundingRect().width()){
            x = parentItem()->boundingRect().width();
        }
        if(y < 0){
            y = 0;
        }
        if(y > parentItem()->boundingRect().height()){
            y = parentItem()->boundingRect().height();
        }
        setPos(x, y);

        if(state == GraphicItemState::EDITING){
            point->setPosition(x, y);
            emit moveTmpEditPathPoint();
        } else if(state == GraphicItemState::EDITING_PERM){
            emit editedPointPositionChanged(x, y);
        }
    }
}

void PointView::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    if(state == GraphicItemState::EDITING){
        float x = pos().x() + pixmap().width()*SCALE/2;
        float y = pos().y() + pixmap().height()*SCALE;
        qDebug() << "to" << x << y;
        emit pathPointChanged(x, y, this);
        QGraphicsPixmapItem::mouseReleaseEvent(event);
    }
    else if(state == GraphicItemState::EDITING_PERM)
        QGraphicsPixmapItem::mouseReleaseEvent(event);
}

void PointView::hoverEnterEvent(QGraphicsSceneHoverEvent * /* unused */){
    setToolTip(point->getName());
    PixmapType tmpType = type;

    setPixmap(PointView::PixmapType::MID);
    /// we don't want to come back to this type but to the type just before we entered the event so we send this signal so the mapView resestablishes the right types
    emit hoverEventSignal(tmpType, this);
}

void PointView::hoverLeaveEvent(QGraphicsSceneHoverEvent * /* unused */){
    QGraphicsPixmapItem::setPixmap(lastPixmap);
}

void PointView::setPos(const qreal x, const qreal y){
    QGraphicsPixmapItem::setPos(x - pixmap().width()*SCALE/2,
           y - pixmap().height()*SCALE);
}

void PointView::setPixmap(const PixmapType pixType){

    lastPixmap = this->pixmap();
    if(type == PointView::HOVER && pixType != PointView::HOVER)
        qDebug() << "no orange anymore";
    type = pixType;
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
