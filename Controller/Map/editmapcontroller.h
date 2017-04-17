#ifndef EDITMAPCONTROLLER_H
#define EDITMAPCONTROLLER_H

#include <QObject>
#include <QVector>
#include <QColor>

class QQuickItem;
class QQmlApplicationEngine;
class EditMapPaintedItem;

class EditMapController : public QObject {

        Q_OBJECT

public:

    EditMapController(QQmlApplicationEngine* engine, QObject *applicationWindow, QObject* parent);

    EditMapPaintedItem* getPaintedItem() const { return paintedItem; }

public slots:
    /**
     * @brief add_item
     * @param shape
     * @param color
     * @param thickness
     * @param x
     * @param y
     * @param _update
     * add a new item (LINE, POINT or SQUARE depending on the <shape)
     * at position (x, y), if <_update> is set to true it means
     * the item already exists and we only need to update it
     */
    void add_item(int shape, QColor color, int thickness, int x, int y, bool _update);
    /**
     * @brief clearMapItems
     * remove all the items and clear the undo vector as well
     */
    void clearMapItems();

private:
    EditMapPaintedItem* paintedItem;

};

#endif /// EDITMAPCONTROLLER_H
