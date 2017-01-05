#ifndef SCANMAPGRAPHICSITEM_H
#define SCANMAPGRAPHICSITEM_H

#include <QGraphicsPixmapItem>

class ScanMapGraphicsItem: public QObject, public QGraphicsPixmapItem {
    Q_OBJECT
public:
    ScanMapGraphicsItem(QString robotName);
    void updateRobotPos(double x, double y, double ori);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

signals:
    void pixmapClicked();
    void robotGoTo(double, double);

private:
    QPointF dragStartPosition;
    QGraphicsPixmapItem* scanRobotView;
};

#endif // SCANMAPGRAPHICSITEM_H
