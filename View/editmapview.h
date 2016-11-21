#ifndef EDITMAPVIEW_H
#define EDITMAPVIEW_H

#include <QObject>
#include <QGraphicsItem>

class EditMapView: public QObject, public QGraphicsItem {
    Q_OBJECT
public:
    EditMapView(int _width, int _height, QGraphicsItem *parent = Q_NULLPTR);
    QRectF boundingRect() const Q_DECL_OVERRIDE;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *) Q_DECL_OVERRIDE;

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) Q_DECL_OVERRIDE;

signals:
    void draw(int, int);

private:
    QPointF dragStartPosition;
    int width;
    int height;
};

#endif // EDITMAPVIEW_H
