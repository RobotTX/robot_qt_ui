#include "pointview.h"
#include "Model/point.h"
#include <QGraphicsSceneMouseEvent>
#include <QDebug>
#include <QPixmap>
#include <QMouseEvent>


PointView::PointView(const std::shared_ptr<Point> &_point, QGraphicsItem *parent) :
  QGraphicsPixmapItem(QPixmap(PIXMAP_NORMAL), parent), state(GraphicItemState::NO_STATE), type(PixmapType::NORMAL), lastType(PixmapType::NORMAL){
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
    qDebug() << "PointView::mousePressEvent" << point->getName() << "state " << state;
    if(state == GraphicItemState::NO_STATE){
        qDebug() << "PointView::mousePressEvent NO_STATE";
        if(event->button() == Qt::RightButton){
            emit pointRightClicked(this->getPoint()->getName());
        }
        if(event->button() == Qt::LeftButton){
            emit pointLeftClicked(this->getPoint()->getName());
        }
    } else if(state == GraphicItemState::CREATING_PATH){
        qDebug() << "PointView::mousePressEvent CREATING_PATH";
        addedToPath = true;
        emit addPointPath(getPoint()->getName(), getPoint()->getPosition().getX(), getPoint()->getPosition().getY());
    } else if(state == GraphicItemState::EDITING_PATH){
        qDebug() << "PointView::mousePressEvent EDITING_PATH" << pos().x() << pos().y();
    }  else if(state == GraphicItemState::EDITING_PERM){
        qDebug() << "PointView::mousePressEvent EDITING_PERM" << pos().x() << pos().y();
    } else if(state == GraphicItemState::EDITING_HOME){
        qDebug() << "PointView::mousePressEvent EDITING_HOME";
        emit homeEdited(this->getPoint()->getName());
    } else {
        qDebug() << "PointView::mousePressEvent NO_EVENT";
    }
}

void PointView::mouseMoveEvent(QGraphicsSceneMouseEvent *event){

    if(state == GraphicItemState::EDITING_PATH || state == GraphicItemState::EDITING_PERM){
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

        if(state == GraphicItemState::EDITING_PATH){
            point->setPosition(x, y);
            emit moveTmpEditPathPoint();
        } else if(state == GraphicItemState::EDITING_PERM){
            emit editedPointPositionChanged(x, y);
        }
    }
}

void PointView::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    if(state == GraphicItemState::EDITING_PATH){
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
    emit hoverEventSignal(tmpType, this->getPoint()->getName());
}

void PointView::hoverLeaveEvent(QGraphicsSceneHoverEvent * /* unused */){
    QGraphicsPixmapItem::setPixmap(lastPixmap);
    updatePos();
}

void PointView::setPos(const qreal x, const qreal y){
    qDebug() << "PointView::setPos called"
             << getPoint()->getName() << "from"
             << getPoint()->getPosition().getX()
             << getPoint()->getPosition().getY()
             << "to" << x << y;

    point->setPosition(x, y);
    QGraphicsPixmapItem::setPos(x - pixmap().width()*SCALE/2,
           y - pixmap().height()*SCALE);
}

void PointView::updatePos(void){
    qDebug() << "PointView::updatePos called"
             << getPoint()->getName() << "from"
             << getPoint()->getPosition().getX()
             << "to" << getPoint()->getPosition().getY();

    float x = getPoint()->getPosition().getX();
    float y = getPoint()->getPosition().getY();
    QGraphicsPixmapItem::setPos(x - pixmap().width()*SCALE/2,
           y - pixmap().height()*SCALE);
}

void PointView::setPixmap(const PixmapType pixType){
    qDebug() << "PointView::setPixmap called" << getPoint()->getName();

    lastPixmap = pixmap();
    if(type == PointView::HOVER && pixType != PointView::HOVER)
        type = pixType;
    QPixmap pixmap2;
    if(point->isHome()){
        switch(pixType){
            case NORMAL:
                pixmap2 = QPixmap(PIXMAP_HOME_NORMAL);
            break;
            case MID:
                pixmap2 = QPixmap(PIXMAP_HOME_MID);
            break;
            case START:
                pixmap2 = QPixmap(PIXMAP_HOME_START);
            break;
            case STOP:
                pixmap2 = QPixmap(PIXMAP_HOME_STOP);
            break;
            case HOVER:
                pixmap2 = QPixmap(PIXMAP_HOME_HOVER);
            break;
            case START_STOP:
                pixmap2 = QPixmap(PIXMAP_HOME_START_STOP);
            break;
            default:
                pixmap2 = QPixmap(PIXMAP_HOME_NORMAL);
            break;
        }
    } else {
        switch(pixType){
            case NORMAL:
                pixmap2 = QPixmap(PIXMAP_NORMAL);
            break;
            case MID:
                pixmap2 = QPixmap(PIXMAP_MID);
            break;
            case START:
                pixmap2 = QPixmap(PIXMAP_START);
            break;
            case STOP:
                pixmap2 = QPixmap(PIXMAP_STOP);
            break;
            case HOVER:
                pixmap2 = QPixmap(PIXMAP_HOVER);
            break;
            case START_STOP:
                pixmap2 = QPixmap(PIXMAP_START_STOP);
            break;
            default:
                pixmap2 = QPixmap(PIXMAP_NORMAL);
            break;
        }
    }
    QGraphicsPixmapItem::setPixmap(pixmap2);
    updatePos();
}
