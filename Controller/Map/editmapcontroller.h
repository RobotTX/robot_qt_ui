#ifndef EDITMAPCONTROLLER_H
#define EDITMAPCONTROLLER_H

#include <QObject>
#include <QVector>
#include <QColor>

class QQuickItem;
class QQmlApplicationEngine;

class EditMapController : public QObject {

        Q_OBJECT

public:
    EditMapController(QQmlApplicationEngine* engine, QObject *applicationWindow, QObject* parent);

public slots:
    void add_item(int shape, QColor color, int thickness, int x, int y);

private:
    QQmlApplicationEngine* _engine;

    QVector<QQuickItem*> items;
    QVector<QQuickItem*> undoItems;
};

#endif /// EDITMAPCONTROLLER_H
