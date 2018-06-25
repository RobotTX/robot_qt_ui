#include "editmappainteditem.h"
#include <QGraphicsItem>
#include <QPainter>
#include <QBrush>
#include <QGraphicsObject>
#include <QStyleOptionGraphicsItem>

EditMapPaintedItem::EditMapPaintedItem(QQuickItem *parent): QQuickPaintedItem(parent) {
    setFlag(QQuickItem::ItemHasContents, true);
}

void EditMapPaintedItem::paint(QPainter *painter){
    QPen pen;
//    painter->rotate(-10);
    qDebug() << "viewport = " << painter->viewport();
    for(int i = 0; i < items.size(); i++){
        pen.setColor(items.at(i).color);
        pen.setWidth(items.at(i).thickness);
        painter->setPen(pen);
        Item it = items.at(i);
        switch(it.shape) {
        case POINT:
        {
            for(int i = 0; i < it.points.size(); i++)
                painter->fillRect(QRect(QPoint(it.points.at(i).x() - it.thickness/2, it.points.at(i).y() - it.thickness/2), QSize(it.thickness, it.thickness)), pen.brush());
            break;
        }
        case LINE:
            if(it.points.size() > 1)
                painter->drawLine(it.points.at(0), it.points.last());
            break;
        case OUTLINE:
            painter->save();
            painter->setPen(QPen(items.at(i).color, items.at(i).thickness, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
            if(it.points.size() > 1) {

                painter->drawRect(QRect(QPoint(it.points.at(0).x(), it.points.at(0).y()),
                                 QPoint(it.points.at(1).x(), it.points.at(1).y())));
            }
            painter->rotate(10);
            painter->restore();
            break;
        case SOLID:
        {
            painter->save();
            painter->translate(-180,290);
            if(it.points.size() > 1){
                QPainterPath path;
                painter->rotate(-20);
//                path.addRoundedRect(QRectF(QPoint(it.points.at(0).x() + 178, it.points.at(0).y() - 300),
//                                           QPoint(it.points.at(1).x() + 178, it.points.at(1).y() - 300)), 0, 0);
                path.addRoundedRect(QRectF(QPoint(it.points.at(0).x(), it.points.at(0).y()),
                                           QPoint(it.points.at(1).x(), it.points.at(1).y())), 0, 0);
                qDebug() << "point top left : " << it.points.at(0).x() << it.points.at(0).y();
                qDebug() << "point bottom right : " << it.points.at(1).x() << it.points.at(1).y();
                qDebug() << "currentPosition = " << path.currentPosition();

                painter->fillPath(path, pen.brush());
            }
            painter->restore();
            break;
        }
        default:
            break;
        }
    }
}

void EditMapPaintedItem::addItem(const SHAPE shape, const QColor color, const int thickness, const int x, const int y, bool _update){
    /// if we don't have any items to draw yet or if we need to create a new one, there is no need to check the last point
    if(items.isEmpty() || !_update){
        qDebug() << "we are here";
        QVector<QPoint> new_points;
        new_points.push_back(QPoint(x, y));
        items.push_back({shape, color, thickness, new_points});
        update();
    }
    /// in this case update == true which means we need to add the point to the last item's points
    /// so that we don't create the same point twice
    else if(items.last().points.last().x() != x || items.last().points.last().y() != y){
        switch(shape) {
        case POINT:
            qDebug() << "case point";
            items.last().points.push_back(QPoint(x, y));
            break;
        case LINE:
            qDebug() << "case line";
            items.last().points.push_back(QPoint(x, y));
            break;
        case OUTLINE:
            qDebug() << "case outline";
            if(items.last().points.size() > 1)
                items.last().points[1] = QPoint(x, y);
            else
                items.last().points.push_back(QPoint(x, y));
            break;
        case SOLID:
            qDebug() << "solid";
            if(items.last().points.size() > 1) {
                items.last().points[1] = QPoint(x, y);
                qDebug() << "x = " << x << " y = " << y;
            } else {
                items.last().points.push_back(QPoint(x, y));
                qDebug() << "x = " << x << " y = " << y;
            }
            break;
        default:
            break;
        }
        update();
    }
}

void EditMapPaintedItem::clearMapItems(){
    items.clear();
    undoItems.clear();
    update();
}

void EditMapPaintedItem::undo(){
    if(items.size() > 0){
        undoItems.push_back(items.takeLast());
        update();
    }
}

void EditMapPaintedItem::redo(){
    if(undoItems.size() > 0){
        items.push_back(undoItems.takeLast());
        update();
    }
}

void EditMapPaintedItem::saveImage(QImage image, QString location){
//    qDebug() << "EditMapController::saveImage saving the image called" << image.size() << location;
    QPainter painter;
    painter.begin(&image);
    paint(&painter);
    painter.end();
//    image.save(location, "PGM");
    #if defined(Q_OS_WIN)
        image.save("C" + location, "PGM");
    #endif

    #if defined(Q_OS_LINUX)
        image.save(location, "PGM");
    #endif

    #if defined(Q_OS_MAC)
        image.save(location, "PGM");
    #endif
}
