#include "pointview.h"
#include "Model/point.h"
#include <QGraphicsSceneMouseEvent>
#include <QDebug>
#include <QPixmap>
#include <QMouseEvent>
#include "View/robotview.h"
#include "Model/robot.h"


PointView::PointView(const QSharedPointer<Point> &_point, QGraphicsItem *parent) :
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
    selectedRobot = NULL;
}

void PointView::mousePressEvent(QGraphicsSceneMouseEvent *event){
    qDebug() << "PointView::mousePressEvent" << point->getName() << "state " << state;

    if(state == GraphicItemState::NO_STATE){
        qDebug() << "PointView::mousePressEvent NO_STATE";
        if(event->button() == Qt::RightButton){
            emit pointRightClicked(getPoint()->getName());
        }
        if(event->button() == Qt::LeftButton){
            qDebug() << "PointView::mousePressEvent blabla" << getPoint()->isPath();
            emit pointLeftClicked(getPoint()->getName(),
                                  getPoint()->getPosition().getX(),
                                  getPoint()->getPosition().getY());
        }

    } else if(state == GraphicItemState::ROBOT_CREATING_PATH){
        qDebug() << "PointView::mousePressEvent CREATING_PATH";
        addedToPath = true;
        emit addPointPath(getPoint()->getName(), getPoint()->getPosition().getX(), getPoint()->getPosition().getY(), GraphicItemState::ROBOT_CREATING_PATH);
    } else if(state == GraphicItemState::NO_ROBOT_CREATING_PATH){
        qDebug() << "PointView::mousePressEvent NO_ROBOT_CREATING_PATH";
        emit addPointPath(getPoint()->getName(), getPoint()->getPosition().getX(), getPoint()->getPosition().getY(), GraphicItemState::NO_ROBOT_CREATING_PATH);
    } else if(state == GraphicItemState::ROBOT_EDITING_PATH || state == GraphicItemState::NO_ROBOT_EDITING_PATH){
        qDebug() << "PointView::mousePressEvent " << state << pos().x() << pos().y();

    }  else if(state == GraphicItemState::EDITING_PERM){
        qDebug() << "PointView::mousePressEvent EDITING_PERM" << pos().x() << pos().y();

    } else if(state == GraphicItemState::EDITING_HOME){
        qDebug() << "PointView::mousePressEvent EDITING_HOME";
        float x = pos().x() + pixmap().width()*SCALE/2;
        float y = pos().y() + pixmap().height()*SCALE;
        emit editedHomePositionChanged(x, y, point->getName());

    } else {
        qDebug() << "PointView::mousePressEvent NO_EVENT";
    }
}

void PointView::mouseMoveEvent(QGraphicsSceneMouseEvent *event){

    if(state == GraphicItemState::NO_ROBOT_EDITING_PATH || state == GraphicItemState::ROBOT_EDITING_PATH || state == GraphicItemState::EDITING_PERM || state == GraphicItemState::EDITING_HOME){
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

        if(state == GraphicItemState::ROBOT_EDITING_PATH){
            point->setPosition(x, y);
            emit moveEditedPathPoint(GraphicItemState::ROBOT_EDITING_PATH);
        } else if(state == GraphicItemState::NO_ROBOT_EDITING_PATH){
            point->setPosition(x, y);
            emit moveEditedPathPoint(GraphicItemState::NO_ROBOT_EDITING_PATH);
        } else if(state == GraphicItemState::EDITING_PERM){
            emit editedPointPositionChanged(x, y);
        } else if(state == GraphicItemState::EDITING_HOME){
            if(!point->getName().compare(TMP_POINT_NAME)){
                emit editedHomePositionChanged(x, y, point->getName());
                qDebug() << point->getName() << "being dragged";
            }
        }
    }
}

void PointView::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    if(state == GraphicItemState::ROBOT_EDITING_PATH || state == GraphicItemState::EDITING_PERM || state == GraphicItemState::EDITING_HOME
            || state == GraphicItemState::NO_ROBOT_EDITING_PATH){
        QGraphicsPixmapItem::mouseReleaseEvent(event);
        qDebug() << "mousereleaseseventcalled on point view with state" << state;
    }
}

void PointView::hoverEnterEvent(QGraphicsSceneHoverEvent * /* unused */){
    if(point->getName().compare(TMP_POINT_NAME)){
        setToolTip(point->getName());
    } else
        setToolTip("This point is only temporary,\nto save it permanently click a valid spot\non the map and click the \"+\" button");
    setPixmap(PointView::PixmapType::HOVER, selectedRobot);
    //qDebug() << "hoverEnterEvent : " << lastPixmap
}

void PointView::hoverLeaveEvent(QGraphicsSceneHoverEvent * /* unused */){
    setPixmap(lastType, selectedRobot);
    updatePos();
    if(state == GraphicItemState::ROBOT_CREATING_PATH || state == GraphicItemState::ROBOT_EDITING_PATH
    || state == GraphicItemState::NO_ROBOT_CREATING_PATH || state == GraphicItemState::NO_ROBOT_EDITING_PATH)
        emit updatePathPainterPointView();
}

void PointView::setPos(const qreal x, const qreal y){
    /*qDebug() << "PointView::setPos called"
             << getPoint()->getName() << "from"
             << getPoint()->getPosition().getX()
             << getPoint()->getPosition().getY()
             << "to" << x << y;*/

    point->setPosition(x, y);
    QGraphicsPixmapItem::setPos(x - pixmap().width()*SCALE/2,
           y - pixmap().height()*SCALE);
}

void PointView::updatePos(void){
    float x = getPoint()->getPosition().getX();
    float y = getPoint()->getPosition().getY();
    QGraphicsPixmapItem::setPos(x - pixmap().width() * SCALE/2, y - pixmap().height() * SCALE);
}

void PointView::setPixmap(const PixmapType pixType, RobotView* _selectedRobot){

    lastType = type;
    selectedRobot = _selectedRobot;

    if(pixType != PixmapType::HOVER)
        type = pixType;

    QPixmap pixmap2;
    if(((_selectedRobot && _selectedRobot->getRobot()->getHome() && _selectedRobot->getRobot()->getHome()->getPoint()->getName().compare(point->getName()) == 0)
            || (!_selectedRobot && point->isHome()))){
        qDebug() << "pixmap home set for" << point->getName();
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
            case SELECTED:
                pixmap2 = QPixmap(PIXMAP_HOME_SELECTED);
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
            case SELECTED:
                pixmap2 = QPixmap(PIXMAP_SELECTED);
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


void PointView::setToolTip(const QString toolTip){
    QString name = toolTip;
    if(name.indexOf(PATH_POINT_NAME) == 0){
        name.remove(PATH_POINT_NAME);
        if(name.toInt()){
            QGraphicsPixmapItem::setToolTip("Point number " + name);
        } else {
            QGraphicsPixmapItem::setToolTip(toolTip);
        }
    } else {
        QGraphicsPixmapItem::setToolTip(toolTip);
    }
}
