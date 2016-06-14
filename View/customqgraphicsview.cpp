#include "customqgraphicsview.h"
#include <QWheelEvent>

CustomQGraphicsView::CustomQGraphicsView ( QWidget * parent ) : QGraphicsView(parent) {
    numScheduledScalings = 0;
}

CustomQGraphicsView::CustomQGraphicsView ( QGraphicsScene * scene, QWidget * parent): QGraphicsView(scene, parent) {
    numScheduledScalings = 0;
}

void CustomQGraphicsView::wheelEvent(QWheelEvent *event){
    if(event->delta() > 0){
        scale(1.3, 1.3);
    } else {
        scale(0.7,0.7);
    }
}
