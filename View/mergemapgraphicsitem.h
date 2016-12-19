#ifndef MERGEMAPGRAPHICSITEM_H
#define MERGEMAPGRAPHICSITEM_H

#include <QGraphicsPixmapItem>

/**
 * @brief The MergeMapGraphicsItem class
 * The QGraphicsPixmapItem used in the mergeMapWidget,
 * reimplemented so that we can distinguish a click from a drag and drop
 */
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
