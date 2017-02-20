#include "mapview.h"
#include <QMainWindow>
#include <QSharedPointer>
#include <QMouseEvent>
#include <QDebug>
#include <QGraphicsSceneMouseEvent>
#include "Controller/mainwindow.h"
#include "Model/Map/map.h"
#include "Model/Points/points.h"
#include "Model/Points/point.h"
#include "View/Map/drawobstacles.h"
#include "View/Points/pointview.h"
#include "View/Points/pointview.h"


MapView::MapView (const QPixmap& pixmap, const QSize _size) : QGraphicsPixmapItem(pixmap), size(_size), tmpPointView(Q_NULLPTR), state(GraphicItemState::NO_STATE) {
    /// Tell the class which mouse button to accept
    setAcceptedMouseButtons(Qt::LeftButton | Qt::MidButton);
    /// To drag & drop the map
    setFlag(QGraphicsItem::ItemIsMovable);
}

void MapView::mousePressEvent(QGraphicsSceneMouseEvent *event){
    if(event->button() == Qt::LeftButton){
        /// On mouse press event, we set the drag start position
        dragStartPosition = this->pos();
        QGraphicsPixmapItem::mousePressEvent(event);

    } else if(event->button() == Qt::MidButton)
        emit testCoord(event->pos().x(), event->pos().y());
}

void MapView::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    if(event->button() == Qt::LeftButton){
        float x = dragStartPosition.x() - this->pos().x();
        float y = dragStartPosition.y() - this->pos().y();

        /// we compare the start position of the drag event & the drop position
        /// if we have moved for more than 10 pixels, it's a drag, else it's a click
        /// and we create a temporary point
        if (abs(x) <= 10 && abs(y) <= 10){
            /// click
            if(state == GraphicItemState::NO_STATE){
                tmpPointView->show();
                tmpPointView->setPos(event->pos().x(), event->pos().y());
                emit leftClick();
            }

            else if(state == GraphicItemState::CREATING_PATH)
                emit addPathPoint(PATH_POINT_NAME, event->pos().x(), event->pos().y());

            else if(state == GraphicItemState::EDITING_PERM)
                /// to notify the point information menu that the position has changed and so the point can be displayed at its new position
                emit newCoordinates(event->pos().x(), event->pos().y());

            else if(state == GraphicItemState::EDITING_PATH)
                /// to notify that a point which belongs to the path of a robot has been changed
                emit newCoordinatesPathPoint(event->pos().x(), event->pos().y());
        }
        /// else drag
        QGraphicsPixmapItem::mouseReleaseEvent(event);
    }
}

