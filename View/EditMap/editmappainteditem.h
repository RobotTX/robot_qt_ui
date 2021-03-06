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
    /**
     * @brief clearMapItems
     * removes all the items from the map and clear the undo vector as well
     */
    void clearMapItems();

    void paint(QPainter *painter) Q_DECL_OVERRIDE;

public slots:
    void undo();
    void redo();
    void saveImage(QImage image, QString location);
    void orientationMap(int orientation);

private:
    QVector<Item> items;
    QVector<Item> undoItems;
    int orientationMapValue;
};

#endif /// EDITMAPPAINTEDITEM_H
