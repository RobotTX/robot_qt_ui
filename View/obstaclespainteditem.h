#ifndef OBSTACLESPAINTEDITEM_H
#define OBSTACLESPAINTEDITEM_H


#include <QtQuick/QQuickPaintedItem>

class ObstaclesPaintedItem : public QQuickPaintedItem {

    Q_OBJECT

    Q_PROPERTY(float orientation_ READ orientation WRITE setOrientation NOTIFY orientationChanged)

public:

    ObstaclesPaintedItem(QQuickItem* parent = 0);

    void paint(QPainter *painter) Q_DECL_OVERRIDE;

    void setObstacles(const QVector<QPointF>& obs);

    float orientation() const { return orientation_; }

    void setOrientation(const float orientation) { orientation_ = orientation; }

signals:
    void orientationChanged();

private:
    QVector<QPointF> obstacles_;
    float orientation_;
};

#endif /// OBSTACLESPAINTEDITEM_H
