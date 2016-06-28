#include "customqgraphicsview.h"
#include <QWheelEvent>

CustomQGraphicsView::CustomQGraphicsView ( QWidget * parent ) : QGraphicsView(parent) {
    zoomCoeff = 1.0;
}

CustomQGraphicsView::CustomQGraphicsView ( QGraphicsScene * scene, QWidget * parent): QGraphicsView(scene, parent) {
    zoomCoeff = 1.0;
}

void CustomQGraphicsView::wheelEvent(QWheelEvent *event){
    if(event->delta() > 0){
        if(zoomCoeff < 4){
            scale(1.3, 1.3);
            zoomCoeff *= 1.3;
        }
    } else {
        if(zoomCoeff > 0.6){
            scale(0.7,0.7);
            zoomCoeff *= 0.7;
        }
    }
}
