#ifndef RECOVERPOSITIONMAPGRAPHICSITEM_H
#define RECOVERPOSITIONMAPGRAPHICSITEM_H

#include <QGraphicsPixmapItem>


class RecoverPositionMapGraphicsItem : public QObject, public QGraphicsPixmapItem {

    Q_OBJECT

public:
    explicit RecoverPositionMapGraphicsItem();

signals:

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

public slots:

private:
    QGraphicsPixmapItem* scanRobotView;
};

#endif /// RECOVERPOSITIONMAPGRAPHICSITEM_H
