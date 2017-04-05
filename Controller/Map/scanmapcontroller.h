#ifndef SCANMAPCONTROLLER_H
#define SCANMAPCONTROLLER_H

class MainController;
class QQmlApplicationEngine;

#include <QObject>
#include <QImage>
#include <QVariant>

class ScanMapController : public QObject  {
    Q_OBJECT
public:
    ScanMapController(MainController* parent, QQmlApplicationEngine* engine, QObject *applicationWindow);
    void receivedScanMap(QString ip, QImage map, QString resolution);

signals:
    void receivedScanMap(QVariant);

private:
    QQmlApplicationEngine* engine;
    QObject* applicationWindow;
};

#endif // SCANMAPCONTROLLER_H
