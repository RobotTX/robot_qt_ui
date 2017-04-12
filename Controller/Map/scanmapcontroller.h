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

    void removeMap(QString ip);


private slots:
    void rotateMap(int angle, QString ip);
    void saveScanSlot(QString file_name);
    void resetScanMaps();
    void sendGoalSlot(QString ip, double x, double y);

signals:
    void receivedScanMap(QVariant);
    void readyToBeGrabbed(QVariant);
    void sendGoal(QString, double, double);
    void invalidGoal();

private:
    QMap<QString, ScanMapPaintedItem*> paintedItems;
    /// to store the color of the maps
    QMap<QString, int> colors;
    QQmlApplicationEngine* engine;
    QObject* applicationWindow;
};

#endif /// SCANMAPCONTROLLER_H
