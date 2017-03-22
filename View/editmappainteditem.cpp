#include "editmappainteditem.h"
#include <QGraphicsItem>
#include <QPainter>
#include <QBrush>

EditMapPaintedItem::EditMapPaintedItem(QQuickItem *parent): QQuickPaintedItem(parent) {
    setFlag(QQuickItem::ItemHasContents, true);
}

void EditMapPaintedItem::paint(QPainter *painter){
    QPen pen(_color, 2);
    painter->setPen(pen);
    painter->setRenderHints(QPainter::Antialiasing, true);

    switch(_shape) {
    case POINT:
    {
        QBrush brush;
        brush.setStyle(Qt::BrushStyle::SolidPattern);
        brush.setColor(_color);
        painter->fillRect(QRect(QPoint(_x-_thickness/2, _y-_thickness/2), QSize(_thickness, _thickness)), brush);
        qDebug() << " drew a point " << _x << _y << _thickness;
        break;
    }
    case LINE:
        painter->drawLine(QPointF(403, 40), QPointF(600, 600));
        qDebug() << " drew a line" ;
        break;
    case OUTLINE:
        painter->drawEllipse(QPoint(70, 70), 5, 5);
        qDebug() << " drew a circle";
        break;
    case SOLID:
        break;
    default:
        break;
    }

}
