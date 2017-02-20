#include "customqgraphicsview.h"
#include <QWheelEvent>

CustomQGraphicsView::CustomQGraphicsView ( QGraphicsScene * scene, QWidget * parent)
    : QGraphicsView(scene, parent), zoomCoeff(1.0), dragStartPosition(QPointF(0, 0)){
}

/// to implement a zoom on the map
void CustomQGraphicsView::wheelEvent(QWheelEvent *event){
    /// zooms in
    if(event->delta() > 0){
        if(zoomCoeff < 15){
            scale(1.3, 1.3);
            zoomCoeff *= 1.3;
        }
    } else {
        /// zooms out
        if(zoomCoeff > 0.6){
            scale(1/1.3,1/1.3);
            zoomCoeff *= 1/1.3;
        }
    }
}

void CustomQGraphicsView::keyPressEvent(QKeyEvent* event){
    emit dirKeyPressed(event->key());
    QGraphicsView::keyPressEvent(event);
}

void CustomQGraphicsView::mousePressEvent(QMouseEvent *event){
    /// The starting position when we want to move the view
    if(catchKeyEvent && !itemAt(event->pos()))
        dragStartPosition = event->pos();

    QGraphicsView::mousePressEvent(event);
}

void CustomQGraphicsView::mouseMoveEvent(QMouseEvent *event){
    if(catchKeyEvent && dragStartPosition != QPointF(0, 0)){
        QList<QGraphicsItem *> itemList = items();
        /// Make it look like we move the view but we actually move the items in the view
        /// Only move the items without a parent as if they had a parent, they will be moved when their parent is moved
        for(int i = 0; i < itemList.count(); i++)
            if(itemList.at(i)->parentItem() == NULL)
                itemList.at(i)->moveBy((event->pos().x() - dragStartPosition.x())/zoomCoeff, (event->pos().y() - dragStartPosition.y())/zoomCoeff);
        dragStartPosition = event->pos();
    }
    QGraphicsView::mouseMoveEvent(event);
}

void CustomQGraphicsView::mouseReleaseEvent(QMouseEvent *event){
    /// Reset dragStartPosition so that when mouseMoveEvent, we only move if dragStartPosition has been init before
    if(catchKeyEvent)
        dragStartPosition = QPointF(0, 0);

    QGraphicsView::mouseReleaseEvent(event);
}
