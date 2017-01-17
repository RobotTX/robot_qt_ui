#ifndef DRAWOBSTACLES_H
#define DRAWOBSTACLES_H

class Robot;
class Robots;

#include <QObject>
#include <QGraphicsItem>
#include <QGraphicsSceneMouseEvent>

class DrawObstacles : public QObject, public QGraphicsItem {
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)

public:
    explicit DrawObstacles(const QSize _size, QSharedPointer<Robots> _robots, QGraphicsItem *parent = Q_NULLPTR);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *) Q_DECL_OVERRIDE;

protected:
    /**
     * @brief boundingRect
     * @return the size of the rectangle in which the items will be drawn
     * if the rectangle returned is too small the items might disappear
     * have to redefine this one correctly in order not to lose the items you draw (for example when you zoom)
     */
    QRectF boundingRect() const Q_DECL_OVERRIDE;

private:
    QSize size;
    QSharedPointer<Robots> robots;
};

#endif /// DRAWOBSTACLES_H
