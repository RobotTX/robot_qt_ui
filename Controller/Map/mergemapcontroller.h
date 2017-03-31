#ifndef MERGEMAPCONTROLLER_H
#define MERGEMAPCONTROLLER_H

#include <QObject>
#include <QVector>

class QQmlApplicationEngine;
class MergeMapsPaintedItem;

class MergeMapController : public QObject {

    Q_OBJECT

public:

    MergeMapController(QObject* parent, QQmlApplicationEngine *engine, QObject *applicationWindow);

private slots:

    void importMap(const QString &_filename);
    void exportMap(QString fileName);
    void rotateMap(int angle, int index);
    void removeMap(int index);
    void resetMergeMapWidget();

private:
    QQmlApplicationEngine* _engine;
    QObject* _applicationWindow;
    QVector<MergeMapsPaintedItem*> paintedItems;
};

#endif /// MERGEMAPCONTROLLER_H
