#ifndef MERGEMAPCONTROLLER_H
#define MERGEMAPCONTROLLER_H

#include <QObject>
#include <QVector>

class QQmlApplicationEngine;
class MergeMapsPaintedItem;

class MergeMapController : public QObject {

    Q_OBJECT

public:

    MergeMapController(QQmlApplicationEngine *engine, QObject *applicationWindow, QObject* parent);

private slots:

    void importMap(QString _filename);
    void rotateMap(int angle, int index);
    void removeMap(int index);

private:
    QQmlApplicationEngine* _engine;
    QObject* _applicationWindow;
    QVector<MergeMapsPaintedItem*> paintedItems;
};

#endif /// MERGEMAPCONTROLLER_H
