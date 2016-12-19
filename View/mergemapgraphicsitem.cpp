#include "mergemapgraphicsitem.h"
#include <QDebug>

MergeMapGraphicsItem::MergeMapGraphicsItem() : QGraphicsPixmapItem(){

    /// Tell the class which mouse button to accept
    setAcceptedMouseButtons(Qt::LeftButton);
    /// To drag & drop the map
    setFlag(QGraphicsItem::ItemIsMovable);
}

void MergeMapGraphicsItem::mousePressEvent(QGraphicsSceneMouseEvent *event){
    /// On mouse press event, we set the drag start position
    dragStartPosition = this->pos();
    QGraphicsPixmapItem::mousePressEvent(event);
}

void MergeMapGraphicsItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){

    float x = dragStartPosition.x() - this->pos().x();
    float y = dragStartPosition.y() - this->pos().y();

    /// we compare the start position of the drag event & the drop position
    /// if we have moved for more than 1 pixel, it's a drag, else it's a click
    /// and we create a temporary point
    if (abs(x) <= 1 && abs(y) <= 1){
        /// click
        emit pixmapClicked();
    }
    /// drag and drop
    QGraphicsPixmapItem::mouseReleaseEvent(event);
}
