#ifndef MERGEMAPGRAPHICSITEM_H
#define MERGEMAPGRAPHICSITEM_H

#include <QGraphicsPixmapItem>

class MergeMapGraphicsItem : public QObject, public QGraphicsPixmapItem {
    Q_OBJECT
public:
    MergeMapGraphicsItem();

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

signals:
    void pixmapClicked();

private:
    QPointF dragStartPosition;
};

#endif // MERGEMAPGRAPHICSITEM_H
