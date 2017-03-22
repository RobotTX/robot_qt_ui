#include "editmappainteditem.h"
#include <QGraphicsItem>
#include <QPainter>

EditMapPaintedItem::EditMapPaintedItem(QQuickItem *parent): QQuickPaintedItem(parent) {
    setFlag(QQuickItem::ItemHasContents, true);
}

void EditMapPaintedItem::paint(QPainter *painter){
    QPen pen(_color, 2);
    painter->setPen(pen);
    painter->setRenderHints(QPainter::Antialiasing, true);
    painter->drawPoint(50, 50);
}
