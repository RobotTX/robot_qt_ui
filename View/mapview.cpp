#include "mapview.h"
#include "View/pointview.h"
#include "Model/points.h"
#include "Model/point.h"
#include <QMainWindow>
#include <QMouseEvent>
#include <QDebug>
#include <QGraphicsSceneMouseEvent>
#include "mainwindow.h"
#include "Model/map.h"


MapView::MapView (const QPixmap& pixmap, const QSize _size, std::shared_ptr<Map> _map, QMainWindow* _mainWindow) :
    QGraphicsPixmapItem(pixmap), size(_size), state(GraphicItemState::NO_STATE), mainWindow(_mainWindow), map(_map), idTmp(0){
    /// Tell the class which mouse button to accept
    setAcceptedMouseButtons(Qt::LeftButton);

    /// To drag & drop the map
    setFlag(QGraphicsItem::ItemIsMovable);
    connect(this, SIGNAL(pointLeftClicked()), mainWindow, SLOT(setSelectedPoint()));

}


void MapView::mousePressEvent(QGraphicsSceneMouseEvent *event){
    /// On mouse press event, we set the drag start position
    dragStartPosition = this->pos();
    QGraphicsPixmapItem::mousePressEvent(event);
}

void MapView::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    qDebug() << "MapView::mouseReleaseEvent called";

    float x = dragStartPosition.x() - this->pos().x();
    float y = dragStartPosition.y() - this->pos().y();
    /// we compare the start position of the drag event & the drop position
    /// if we have moved for more than 10 pixels, it's a drag, else it's a click
    /// and we create a temporary point
    if (abs(x) <= 10 && abs(y) <= 10){
        /// click
        if(state == GraphicItemState::NO_STATE){
            qDebug() << "MapView::mouseReleaseEvent NO_STATE";
            std::shared_ptr<PointView> tmpPointView = points->getTmpPointView();
            tmpPointView->show();
            /// might be useless code
            tmpPointView->setPos(event->pos().x(), event->pos().y());
            emit pointLeftClicked();
        } else if(state == GraphicItemState::CREATING_PATH){
            qDebug() << "MapView::mouseReleaseEvent CREATING_PATH";
            /// if it's not a white point of the map we cannot add it to the path
            if(map->getMapImage().pixelColor(event->pos().x()-tmpPointPixmap.width()/2, event->pos().y()-tmpPointPixmap.height()).red() >= 254){
                emit addPathPoint(PATH_POINT_NAME, event->pos().x(), event->pos().y());
           } else {
               emit newMessage("You cannot create a point here because your robot cannot go there. You must click known areas of the map");
               qDebug() << "sorry cannot create a point here";
            }
        } else if(state == GraphicItemState::EDITING_PERM){
            qDebug() << "MapView::mouseReleaseEvent EDITING_PERM" << event->pos().x() << event->pos().y();
            /// to notify the point information menu that the position has changed and so the point can be displayed at its new position
            emit newCoordinates(event->pos().x(), event->pos().y());
        } else if(state == GraphicItemState::EDITING_HOME){
            qDebug() << "MapView::mouseReleaseEvent EDITING_HOME";
            std::shared_ptr<PointView> tmpPointView = points->getTmpPointView();
            tmpPointView->show();
            /// might be useless code
            tmpPointView->setPos(event->pos().x(), event->pos().y());
            emit homeEdited(tmpPointView.get());
        } else if(state == GraphicItemState::EDITING_PATH){
            qDebug() << "MapView::mouseReleaseEvent EDITING_PATH" << event->pos().x() << event->pos().y();
            /// to notify that a point which belongs to the path of a robot has been changed
            emit newCoordinatesPathPoint(event->pos().x(), event->pos().y());
        } else {
            qDebug() << "MapView::mouseReleaseEvent NO_EVENT";
        }
    }
    /// else drag
    QGraphicsPixmapItem::mouseReleaseEvent(event);
}
void MapView::setState(const GraphicItemState _state){
    //qDebug() << "mapview setstate" << _state;
    state = _state;
}

 void MapView::setPoints(std::shared_ptr<Points> _points){
    //qDebug() << "MapView::setPoints called" << _points->count();
    points = _points;
 }
