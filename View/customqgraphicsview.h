#ifndef CUSTOMQGRAPHICSVIEW_H
#define CUSTOMQGRAPHICSVIEW_H

class QWheelEvent;
class QtWidgets;

#include <QtWidgets>

/**
 * @brief The CustomQGraphicsView class
 * The view containing the map to display
 */
class CustomQGraphicsView : public QGraphicsView {
    Q_OBJECT
public:
    CustomQGraphicsView(QGraphicsScene *scene, QWidget *parent = 0);
    float getZoomCoeff(void) const { return zoomCoeff; }
    void setZoomCoeff(const float _zoom);

protected:
    /**
     * @brief wheelEvent
     * @param event
     * Overload of the wheelEvent to zoom in and out of the map with the wheel
     */
    void wheelEvent(QWheelEvent *event);

private:
    float zoomCoeff;
};

/// to set a particular zoom on the map
inline void CustomQGraphicsView::setZoomCoeff(const float _zoom){
    //qDebug() << zoomCoeff << "wanted->" << _zoom;
    scale(_zoom/zoomCoeff, _zoom/zoomCoeff);
    zoomCoeff = _zoom;
}

#endif // CUSTOMQGRAPHICSVIEW_H
