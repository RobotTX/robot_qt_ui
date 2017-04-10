#include "scanmappainteditem.h"
#include <QDebug>
#include <QPainter>

ScanMapPaintedItem::ScanMapPaintedItem(QQuickItem *parent) : QQuickPaintedItem(parent),
    left(0), top(0), xRobot(0.0), yRobot(0.0), orientationRobot(0.0) {
    setFlag(QQuickItem::ItemHasContents, true);
    setAcceptedMouseButtons(Qt::LeftButton | Qt::RightButton);
    setTransformOrigin(QQuickItem::Center);
}

void ScanMapPaintedItem::paint(QPainter *painter){
    qDebug() << "ScanMapPaintedItem::paint called" << _image.size();
    painter->drawImage(QPoint(0, 0), _image);
}
