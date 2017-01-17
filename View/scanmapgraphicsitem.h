#ifndef SCANMAPGRAPHICSITEM_H
#define SCANMAPGRAPHICSITEM_H

#include <QGraphicsPixmapItem>

class ScanMapGraphicsItem: public QObject, public QGraphicsPixmapItem {
    Q_OBJECT
public:
    ScanMapGraphicsItem(QString robotName);

    /// Getter
    QGraphicsPixmapItem* getRobotView(void) const { return scanRobotView; }

    /**
     * @brief updateRobotPos
     * @param x
     * @param y
     * @param ori
     * Update the position of the robot on the map
     */
    void updateRobotPos(double x, double y, double ori);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

signals:
    /**
     * @brief pixmapClicked
     * When the pixmap is clicked
     */
    void pixmapClicked();

    /**
     * @brief robotGoTo
     * When using the middle mouse, we tell the robot where to go
     */
    void robotGoTo(double, double);

private:
    QPointF dragStartPosition;
    QGraphicsPixmapItem* scanRobotView;
};

#endif // SCANMAPGRAPHICSITEM_H
