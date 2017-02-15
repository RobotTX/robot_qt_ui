#ifndef RECOVERPOSITIONMAPGRAPHICSITEM_H
#define RECOVERPOSITIONMAPGRAPHICSITEM_H

#include <QGraphicsPixmapItem>

class RecoverPositionMapGraphicsItem : public QObject, public QGraphicsPixmapItem {

    Q_OBJECT

public:
    explicit RecoverPositionMapGraphicsItem(const QString robotName);

    QGraphicsPixmapItem* getRobotView(void) const { return scanRobotView; }

    /**
     * @brief updateRobotPos
     * @param x
     * @param y
     * @param ori
     * Update the position of the robot on the map
     */
    void updateRobotPos(double x, double y, double ori);

signals:
    /**
     * @brief robotGoTo
     * When using the right mouse button, we tell the robot where to go
     */
    void robotGoTo(double, double);

    /**
     * @brief pixmapClicked
     * When the pixmap is clicked
     */
    void pixmapClicked();

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private:
    QPointF dragStartPosition;

    QGraphicsPixmapItem* scanRobotView;
};

#endif /// RECOVERPOSITIONMAPGRAPHICSITEM_H
