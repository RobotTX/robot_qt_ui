#include "scanmapgraphicsitem.h"
#include <QGraphicsSceneMouseEvent>
#include <QDebug>

ScanMapGraphicsItem::ScanMapGraphicsItem(QString robotName) : QGraphicsPixmapItem(){

    /// Tell the class which mouse button to accept
    setAcceptedMouseButtons(Qt::LeftButton | Qt::MidButton);
    /// To drag & drop the map
    setFlag(QGraphicsItem::ItemIsMovable);

    scanRobotView = new QGraphicsPixmapItem(QPixmap(":/icons/final_robot.png"), this);
    scanRobotView->setScale(0.07);
    scanRobotView->setTransformOriginPoint(scanRobotView->pixmap().width()/2, scanRobotView->pixmap().height()/2);
    scanRobotView->setToolTip(robotName);
}

void ScanMapGraphicsItem::mousePressEvent(QGraphicsSceneMouseEvent *event){
    if(event->button() == Qt::LeftButton){
        /// On mouse press event, we set the drag start position
        dragStartPosition = this->pos();
        QGraphicsPixmapItem::mousePressEvent(event);

    } else if(event->button() == Qt::MidButton){
        emit robotGoTo(event->pos().x(), event->pos().y());
    }
}

void ScanMapGraphicsItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){

    if(event->button() == Qt::LeftButton){
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
}

void ScanMapGraphicsItem::updateRobotPos(double x, double y, double ori){
    scanRobotView->setRotation(ori);
    //qDebug() << "ScanMapGraphicsItem::updateRobotPos" << x << y << scanRobotView->pixmap().width()/2 << scanRobotView->pixmap().height()/2 << x-scanRobotView->pixmap().width()/2 << y-scanRobotView->pixmap().height()/2;
    scanRobotView->setPos(x-scanRobotView->pixmap().width()/2, y-scanRobotView->pixmap().height()/2);
}
