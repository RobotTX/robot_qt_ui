#include "editmapview.h"
#include <QDebug>
#include <QGraphicsSceneMouseEvent>

EditMapView::EditMapView(const QPixmap& pixmap) : QGraphicsPixmapItem(pixmap){
    /// Tell the class which mouse button to accept
    setAcceptedMouseButtons(Qt::LeftButton);
    /// To drag & drop the map
    setFlag(QGraphicsItem::ItemIsMovable);
}

void EditMapView::mousePressEvent(QGraphicsSceneMouseEvent *event){
    /// On mouse press event, we set the drag start position
    dragStartPosition = this->pos();
    QGraphicsPixmapItem::mousePressEvent(event);
}

void EditMapView::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){

    float x = dragStartPosition.x() - this->pos().x();
    float y = dragStartPosition.y() - this->pos().y();

    /// we compare the start position of the drag event & the drop position
    /// if we have moved for more than 10 pixels, it's a drag, else it's a click
    /// and we create a temporary point
    if (abs(x) <= 10 && abs(y) <= 10){
        /// click
        qDebug() << "EditMapView::mouseReleaseEvent click" << event->pos().x() << event->pos().y();
    }
    /// else drag
    QGraphicsPixmapItem::mouseReleaseEvent(event);
}
