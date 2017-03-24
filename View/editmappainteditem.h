#ifndef EDITMAPPAINTEDITEM_H
#define EDITMAPPAINTEDITEM_H

#include <QtQuick/QQuickPaintedItem>
#include <QColor>
#include <QMetaEnum>
#include <QImage>

class EditMapPaintedItem : public QQuickPaintedItem {

    Q_OBJECT

public:

    enum SHAPE { POINT, LINE, OUTLINE, SOLID };

    struct Item {
        EditMapPaintedItem::SHAPE shape;
        QColor color;
        int thickness;
        QVector<QPoint> points;
    };

    EditMapPaintedItem(QQuickItem* parent = 0);

    void addItem(const SHAPE shape, const QColor color, const int thickness, const int x, const int y, bool _update);
    void clearMapItems();
/*
    int thickness() const { return _thickness; }
    QColor color() const { return _color; }
    int shape() const { return _shape; }
    int x() const { return _x; }
    int y() const { return _y; }

    void setThickness(const int thick) { _thickness = thick; }
    void setColor(const QColor color) { _color = color; }
    void setShape(const int shape) { _shape = shape; }
    void setX(const int x) { _x = x; }
    void setY(const int y) { _y = y; }
*/
    void paint(QPainter *painter);

public slots:
    void undo();
    void redo();
    void saveImage(QImage image, QString location);

private:
    /*
    int _thickness;
    QColor _color;
    int _shape;
    int _x;
    int _y;
    */
    QVector<Item> items;
    QVector<Item> undoItems;
};

#endif /// EDITMAPPAINTEDITEM_H
