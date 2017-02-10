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
    void setCatchKeyEvent(const bool _catchKeyEvent) { catchKeyEvent = _catchKeyEvent; }

signals:
    void dirKeyPressed(int key);

protected:
    /**
     * @brief wheelEvent
     * @param event
     * Overload of the wheelEvent to zoom in and out of the map with the wheel
     */
    void wheelEvent(QWheelEvent *event);

    /**
     * @brief keyPressEvent
     * @param event
     * when a directional key is pressed, we send a signal to whoever need it
     */
    void keyPressEvent(QKeyEvent* event);

    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);

private:
    float zoomCoeff;
    bool catchKeyEvent;
    QPointF dragStartPosition;
};

/// to set a particular zoom on the map
inline void CustomQGraphicsView::setZoomCoeff(const float _zoom){
    scale(_zoom/zoomCoeff, _zoom/zoomCoeff);
    zoomCoeff = _zoom;
}

#endif /// CUSTOMQGRAPHICSVIEW_H
