#include "customqgraphicsview.h"
#include <QWheelEvent>

CustomQGraphicsView::CustomQGraphicsView ( QGraphicsScene * scene, QWidget * parent)
    : QGraphicsView(scene, parent), zoomCoeff(1.0){

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

