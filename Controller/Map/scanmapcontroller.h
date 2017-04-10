#ifndef SCANMAPCONTROLLER_H
#define SCANMAPCONTROLLER_H

class MainController;
class QQmlApplicationEngine;

#include <QObject>
#include <QImage>
#include <QVariant>
#include "View/scanmappainteditem.h"

class ScanMapController : public QObject  {

    Q_OBJECT

public:

    ScanMapController(MainController* parent, QQmlApplicationEngine* engine, QObject *applicationWindow);
    void receivedScanMap(QString ip, QImage map, QString resolution);

    void updateRobotPos(QString ip, float x, float y, float orientation);

signals:
    void receivedScanMap(QVariant);

private:
    QMap<QString, ScanMapPaintedItem*> paintedItems;
    QQmlApplicationEngine* engine;
    QObject* applicationWindow;
};

#endif /// SCANMAPCONTROLLER_H
