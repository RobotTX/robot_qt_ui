#ifndef EDITMAPVIEW_H
#define EDITMAPVIEW_H

#include <QObject>
#include <QGraphicsItem>
#include <QPainter>
#include <QList>
#include <QVector>

class EditMapView: public QObject, public QGraphicsItem {
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    EditMapView(int _width, int _height, QGraphicsItem *parent = Q_NULLPTR);
    QRectF boundingRect() const Q_DECL_OVERRIDE;
    void paint(QPainter *_painter, const QStyleOptionGraphicsItem *, QWidget *) Q_DECL_OVERRIDE;

    QVector<QPair<QVector<int>, QVector<QPointF>>> getItems(void) const { return items; }
    QVector<QPointF> getLine(QPointF p1, QPointF p2);

protected:
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) Q_DECL_OVERRIDE;
    void mousePressEvent(QGraphicsSceneMouseEvent *event) Q_DECL_OVERRIDE;
    void resetLastPoint();
    QPointF getPoint(float x, float y);

private slots:
    void undoSlot();
    void redoSlot();
    void changeColorSlot(int);
    void changeShapeSlot(int);
    void changeSizeSlot(int);
    void resetSlot();

private:
    QPointF dragStartPosition;
    int width;
    int height;
    int color;
    int shape;
    int size;
    int lastX;
    int lastY;
    int lastSize;
    QVector<QPair<QVector<int>, QVector<QPointF>>> items;
    bool released;
    QVector<QPair<QVector<int>, QVector<QPointF>>> undoItems;
};

#endif // EDITMAPVIEW_H
