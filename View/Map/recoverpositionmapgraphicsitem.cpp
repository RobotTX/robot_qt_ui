#include "recoverpositionmapgraphicsitem.h"
#include <QGraphicsSceneMouseEvent>

RecoverPositionMapGraphicsItem::RecoverPositionMapGraphicsItem(const QString robotName) : QGraphicsPixmapItem()
{
    /// Tell the class which mouse button to accept
    setAcceptedMouseButtons(Qt::LeftButton | Qt::RightButton);

    /// To drag & drop the map
    setFlag(QGraphicsItem::ItemIsMovable);

    /// The view of the robot on the map
    scanRobotView = new QGraphicsPixmapItem(QPixmap(":/icons/final_robot.png"), this);
    scanRobotView->setScale(0.07);
    scanRobotView->setTransformOriginPoint(scanRobotView->pixmap().width()/2, scanRobotView->pixmap().height()/2);
    scanRobotView->setToolTip(robotName);
    scanRobotView->setZValue(2);
}

void RecoverPositionMapGraphicsItem::mousePressEvent(QGraphicsSceneMouseEvent *event){
    if(event->button() == Qt::LeftButton){
        /// On mouse press event, we set the drag start position
        dragStartPosition = this->pos();
        QGraphicsPixmapItem::mousePressEvent(event);

    } else if(event->button() == Qt::RightButton)
        emit robotGoTo(event->pos().x(), event->pos().y());
}

void RecoverPositionMapGraphicsItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    if(event->button() == Qt::LeftButton){
        float x = dragStartPosition.x() - this->pos().x();
        float y = dragStartPosition.y() - this->pos().y();

        /// TODO connect the emit with a slot to select the robot in the list on the left
        /// we compare the start position of the drag event & the drop position
        /// if we have moved for more than 1 pixel, it's a drag, else
        if (abs(x) <= 1 && abs(y) <= 1)
            emit pixmapClicked();

        /// drag and drop
        QGraphicsPixmapItem::mouseReleaseEvent(event);
    }
}

void RecoverPositionMapGraphicsItem::updateRobotPos(double x, double y, double ori){
    scanRobotView->setRotation(ori);
    scanRobotView->setPos(x-scanRobotView->pixmap().width()/2, y-scanRobotView->pixmap().height()/2);
}
