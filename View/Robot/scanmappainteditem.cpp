#include "scanmappainteditem.h"
#include <QDebug>
#include <QPainter>

ScanMapPaintedItem::ScanMapPaintedItem(QQuickItem *parent)
    : QQuickPaintedItem(parent), ip(""), name(""), left(0), top(0), xRobot(0.0), yRobot(0.0), orientationRobot(0.0)
{
    setFlag(QQuickItem::ItemHasContents, true);
    setAcceptedMouseButtons(Qt::LeftButton | Qt::RightButton);
    /// so we rotate around the center of the item and not around the top left corner
    setTransformOrigin(QQuickItem::Center);
}

void ScanMapPaintedItem::paint(QPainter *painter){
//    qDebug() << "ScanMapPaintedItem::paint called" << _image.size();
    painter->drawImage(QPoint(0, 0), _image);
}

void ScanMapPaintedItem::setRobotX(const double x){
    xRobot = x - left;
}

void ScanMapPaintedItem::setRobotY(const double y){
    yRobot = y - top;
}
