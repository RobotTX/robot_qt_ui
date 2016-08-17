#include "customqgraphicsview.h"
#include <QWheelEvent>

CustomQGraphicsView::CustomQGraphicsView ( QGraphicsScene * scene, QWidget * parent)
    : QGraphicsView(scene, parent), zoomCoeff(1.0){

}

/// to implement a zoom on the map
void CustomQGraphicsView::wheelEvent(QWheelEvent *event){
    if(event->delta() > 0){
        if(zoomCoeff < 15){
            scale(1.3, 1.3);
            zoomCoeff *= 1.3;
        }
    } else {
        if(zoomCoeff > 0.6){
            scale(1/1.3,1/1.3);
            zoomCoeff *= 1/1.3;
        }
    }
}

void CustomQGraphicsView::setZoomCoeff(const float _zoom){
    if(_zoom > zoomCoeff){
        while(zoomCoeff < _zoom){
            scale(1.3, 1.3);
            zoomCoeff *= 1.3;
        }
    } else {
        while(zoomCoeff > _zoom){
            scale(1/1.3, 1/1.3);
            zoomCoeff *= 1/1.3;
        }
    }
    qDebug() << "Zoom set" << zoomCoeff;
}
