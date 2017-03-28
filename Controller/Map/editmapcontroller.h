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
    void add_item(int shape, QColor color, int thickness, int x, int y, bool _update);
    void clearMapItems();

private:
    EditMapPaintedItem* paintedItem;

};

#endif /// EDITMAPCONTROLLER_H
