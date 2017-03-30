#include "mergemapspainteditem.h"
#include <QPainter>
#include <QMouseEvent>

MergeMapsPaintedItem::MergeMapsPaintedItem(QQuickItem *parent): QQuickPaintedItem(parent) {
    setFlag(QQuickItem::ItemHasContents, true);
    setAcceptedMouseButtons(Qt::LeftButton);
    setTransformOrigin(QQuickItem::Center);
}

void MergeMapsPaintedItem::paint(QPainter *painter){
    painter->drawImage(QPoint(0, 0), _image);
}

void MergeMapsPaintedItem::rotate(const int angle){
    setRotation(angle);
}
