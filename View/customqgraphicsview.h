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
    CustomQGraphicsView(QWidget *parent = 0);
    CustomQGraphicsView(QGraphicsScene *scene, QWidget *parent = 0);

protected:
    /**
     * @brief wheelEvent
     * @param event
     * Overload of the wheelEvent to zoom in and out of the map with the wheel
     */
    void wheelEvent(QWheelEvent *event);

private:
    int numScheduledScalings;
};

#endif // CUSTOMQGRAPHICSVIEW_H
