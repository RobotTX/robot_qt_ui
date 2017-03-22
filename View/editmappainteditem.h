#ifndef EDITMAPPAINTEDITEM_H
#define EDITMAPPAINTEDITEM_H

#include <QtQuick/QQuickPaintedItem>
#include <QColor>

enum SHAPE { POINT, LINE, OUTLINE, SOLID };

class EditMapPaintedItem : public QQuickPaintedItem {

    Q_OBJECT
    Q_PROPERTY(int thickness READ thickness WRITE setThickness)
    Q_PROPERTY(QColor color READ color WRITE setColor)
    Q_PROPERTY(SHAPE shape READ shape)

public:

    EditMapPaintedItem(QQuickItem* parent = 0);

    int thickness() const { return _thickness; }
    QColor color() const { return _color; }
    SHAPE shape() const { return _shape; }

    void setThickness(const int thick) { _thickness = thick; }
    void setColor(const QColor color) { _color = color; }

    void paint(QPainter *painter);

private:
    int _thickness;
    QColor _color;
    SHAPE _shape;
};

#endif /// EDITMAPPAINTEDITEM_H
