#ifndef SCANMAPGRAPHICSITEM_H
#define SCANMAPGRAPHICSITEM_H

class Robots;

#include <QGraphicsPixmapItem>

class ScanMapGraphicsItem: public QObject, public QGraphicsPixmapItem {

    Q_OBJECT

public:
    ScanMapGraphicsItem(QString robotName, QSharedPointer<Robots> _robots);

    /// Getter
    QGraphicsPixmapItem* getRobotView(void) const { return scanRobotView; }
    QGraphicsPixmapItem* getLaserView(void) const { return laserView; }

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

private slots:
    void updateLaserSLot();

private:
    QPointF dragStartPosition;
    QGraphicsPixmapItem* scanRobotView;
    QGraphicsPixmapItem* laserView;
    QString robotName;
    QSharedPointer<Robots> robots;
};

#endif /// SCANMAPGRAPHICSITEM_H
