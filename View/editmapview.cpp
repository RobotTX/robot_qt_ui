#include "editmapview.h"
#include <QDebug>
#include <QGraphicsSceneMouseEvent>

EditMapView::EditMapView(int _width, int _height, QGraphicsItem* parent) : QGraphicsItem(parent), width(_width), height(_height){
    /// Tell the class which mouse button to accept
    setAcceptedMouseButtons(Qt::LeftButton);

    /// To be able to receive the hover event
    setAcceptHoverEvents(true);

    /// To have this QGraphicsLayer above the other one
    setZValue(2);

    /// To drag & drop the map
    setFlags(ItemIsSelectable | ItemIsMovable);

    color = Qt::black;
    shape = 0;
    size = 1;
    released = true;
    resetLastPoint();
}

void EditMapView::resetLastPoint(){
    lastX = -1;
    lastY = -1;
    lastSize = -1;
}

QRectF EditMapView::boundingRect() const{
    return QRectF(0, 0, width, height);
}

void EditMapView::mousePressEvent(QGraphicsSceneMouseEvent *event){
    /// When we press, we create a new item to draw
    if(!(flags() & QGraphicsItem::ItemIsMovable)){
        qDebug() << "EditMapView::mousePressEvent click" << event->pos().x() << event->pos().y();
        /// If we are using the bucket
        if(shape > 0){
            /// Create the first point when we try to draw a point, line or rectangle
            QVector<QPointF> points;
            QVector<int> conf;
            conf.push_back(color);
            conf.push_back(shape);
            conf.push_back(size);

            undoItems.clear();
            points.push_back(getPoint(event->pos().x(), event->pos().y()));
            items.push_back(QPair<QVector<int>, QVector<QPointF>>(conf, points));
            lastX = static_cast<int>(event->pos().x());
            lastY = static_cast<int>(event->pos().y());
            lastSize = size;
            update();
            return;
        }
    }
    QGraphicsItem::mousePressEvent(event);
}

void EditMapView::mouseMoveEvent(QGraphicsSceneMouseEvent *event){
    if(!(flags() & QGraphicsItem::ItemIsMovable)){
        //qDebug() << "EditMapView::mouseMoveEvent click" << event->pos().x() << event->pos().y();

        /// If we move while pressing and drawing
        if(!(lastX == static_cast<int>(event->pos().x()) && lastY == static_cast<int>(event->pos().y()) && lastSize == size)){
            if(shape > 0){
                QPair<QVector<int>, QVector<QPointF>> pair = items.takeLast();

                /// If we are drawing a line or a rectangle, we only need 2 points so if we already have a second point, we remove it for the new one
                if(shape > 1 && pair.second.size() > 1){
                    pair.second.removeLast();
                }

                pair.second.push_back(getPoint(event->pos().x(), event->pos().y()));
                items.push_back(pair);
                lastX = static_cast<int>(event->pos().x());
                lastY = static_cast<int>(event->pos().y());
                lastSize = size;
                released = false;
                update();
                return;
            }
        }
    }
    QGraphicsItem::mouseMoveEvent(event);
}

void EditMapView::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    if(!(flags() & QGraphicsItem::ItemIsMovable)){
        //qDebug() << "EditMapView::mouseReleaseEvent click" << event->pos().x() << event->pos().y();
        if(shape > 0){
            resetLastPoint();
            released = true;
            update();
            return;
        }
    }
    QGraphicsItem::mouseReleaseEvent(event);
}

void EditMapView::paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget*){
    QPen p = painter->pen();

    for(int i = 0; i < items.size(); i++){
        if(items.at(i).first.size() > 2){
            int _color = items.at(i).first.at(0);
            int _shape = items.at(i).first.at(1);
            int _size = items.at(i).first.at(2);
            QVector<QPointF> points = items.at(i).second;

            //painter->setBrush(Qt::SolidPattern);

            switch(_shape){
                case 0:
                    qDebug() << "EditMapView::paint should not be here (case 0)";
                break;
                case 1:
                    //qDebug() << "EditMapView::paint Drawing points";
                    painter->setPen(QPen(QColor(_color, _color, _color), _size));
                    for(int j = 0; j < points.size(); j++)
                        painter->drawPoint(points.at(j));

                break;
                case 2:{
                    //qDebug() << "EditMapView::paint Drawing a line";
                    painter->setPen(QPen(QColor(_color, _color, _color), _size));
                    if(points.size() > 1){
                        QVector<QPointF> newPoints = getLine(points.at(0), points.at(1));
                        for(int j = 0; j < newPoints.size(); j++)
                            painter->drawPoint(newPoints.at(j));
                    }
                }
                break;
                case 3:
                    //qDebug() << "EditMapView::paint Drawing an empty rectangle";
                    painter->setPen(QPen(QColor(_color, _color, _color), _size, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
                    if(points.size() > 1)
                        painter->drawRect(QRect(QPoint(static_cast<int>(points.at(0).x()), static_cast<int>(points.at(0).y())),
                                                QPoint(static_cast<int>(points.at(1).x()), static_cast<int>(points.at(1).y()))));
                break;
                case 4:
                    //qDebug() << "EditMapView::paint Drawing a filled rectange";
                    if(points.size() > 1){
                        QRect rect = QRect(QPoint(static_cast<int>(points.at(0).x()), static_cast<int>(points.at(0).y())),
                                           QPoint(static_cast<int>(points.at(1).x()), static_cast<int>(points.at(1).y())));
                        painter->setPen(QPen(QColor(_color, _color, _color), qMax(rect.width(), rect.height()), Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
                        painter->fillRect(rect, QColor(_color, _color, _color));
                    }
                break;
                default:
                    qDebug() << "EditMapView::paint should not be here (default)";
                break;
            }
        } else {
            qDebug() << "EditMapView::paint should not be here (items.first < 3)";
        }
    }

    painter->setPen(p);
}

void EditMapView::undoSlot(){
    qDebug() << "EditMapView::undoSlot called";
    if(items.size() > 0){
        undoItems.push_back(items.takeLast());
        update();
    }
}

void EditMapView::redoSlot(){
    qDebug() << "EditMapView::redoSlot called";
    if(undoItems.size() > 0){
        items.push_back(undoItems.takeLast());
        update();
    }
}

void EditMapView::changeColorSlot(int _color){
    qDebug() << "EditMapView::changeColorSlot called" << _color;
    switch(_color){
        case 0:
            color = 0;
        break;
        case 1:
            color = 205;
        break;
        case 2:
            color = 255;
        break;
        default:
            color = 0;
        break;
    }

    resetLastPoint();
}

void EditMapView::changeShapeSlot(int _shape){
    qDebug() << "EditMapView::changeShapeSlot called" << _shape;
    shape = _shape;
    if(!shape)
        setFlag(QGraphicsItem::ItemIsMovable);
    else
        setFlag(QGraphicsItem::ItemIsMovable, false);

    resetLastPoint();
}

void EditMapView::changeSizeSlot(int _size){
    qDebug() << "EditMapView::changeSizeSlot called" << _size;
    size = _size;

    resetLastPoint();
}

QPointF EditMapView::getPoint(float x, float y){
    return QPointF(static_cast<int>(x) + 0.5, static_cast<int>(y) + 0.5);
}

QVector<QPointF> EditMapView::getLine(QPointF p1, QPointF p2){
    int Ax = static_cast<int>(p1.x());
    int Ay = static_cast<int>(p1.y());
    int Bx = static_cast<int>(p2.x());
    int By = static_cast<int>(p2.y());

    QVector<QPointF> res;

    int	y,x,dy,dx,sx,sy;
    int	decision,incE,incNE;

    dx = Bx - Ax;
    dy = By - Ay;

    if(dx >= 0)
        sx = 1;
    else
        sx = -1;
    if(dy >= 0)
        sy = 1;
    else
        sy = -1;

    dx = qAbs(dx);
    dy = qAbs(dy);


    Bx += sx;
    By += sy;

    if(dy > dx) {
        incE = 2 * dx;
        incNE = 2 * dx - 2 * dy;
        decision = 2 * dx - dy;;

        x = Ax;
        y = Ay;
        while(y != By){
            res.push_back(getPoint(x, y));
            if(decision <= 0)
                decision += incE;
            else{
                decision += incNE;
                x += sx;
            }
            y += sy;
        }
    }else{
        incE = 2 * dy;
        incNE = 2 * dy - 2 * dx;
        decision = 2 * dy - dx;

        x = Ax;
        y = Ay;
        while(x != Bx){
            res.push_back(getPoint(x, y));
            if(decision <= 0)
                decision += incE;
            else{
                decision += incNE;
                y += sy;
            }
            x += sx;
        }
    }

    return res;
}
