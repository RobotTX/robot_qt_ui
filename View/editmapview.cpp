#include "editmapview.h"
#include <QDebug>
#include <QGraphicsSceneMouseEvent>
#include <QPainter>

EditMapView::EditMapView(int _width, int _height, QGraphicsItem* parent) : QGraphicsItem(parent), width(_width), height(_height){
    /// Tell the class which mouse button to accept
    setAcceptedMouseButtons(Qt::LeftButton);
    /// To drag & drop the map
    setFlag(QGraphicsItem::ItemIsMovable);
    setZValue(2);
}

QRectF EditMapView::boundingRect() const
{
    return QRectF(0, 0, width, height);
}

void EditMapView::mousePressEvent(QGraphicsSceneMouseEvent *event){
    /// On mouse press event, we set the drag start position
    dragStartPosition = this->pos();
    QGraphicsItem::mousePressEvent(event);
}

void EditMapView::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){

    if(flags() == QGraphicsItem::ItemIsMovable){
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
        QGraphicsItem::mouseReleaseEvent(event);
    } else {
        qDebug() << "EditMapView::mouseReleaseEvent click" << event->pos().x() << event->pos().y();
        emit draw(static_cast<int>(event->pos().x()), static_cast<int>(event->pos().y()));
    }
}

void EditMapView::paint(QPainter *painter, const QStyleOptionGraphicsItem*,
           QWidget*)
{
    QPen p = painter->pen();
    painter->setPen(QPen(Qt::red, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter->setBrush(Qt::NoBrush);
    painter->drawRoundedRect(width/2-10, height/2-10, 100, 100, 20, 20);
    painter->setPen(p);
}
