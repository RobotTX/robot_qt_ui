#include "scanmapcontroller.h"
#include <QQmlApplicationEngine>
#include "Controller/maincontroller.h"

ScanMapController::ScanMapController(MainController* parent, QQmlApplicationEngine* _engine, QObject *_applicationWindow)
    : QObject(parent), engine(_engine), applicationWindow(_applicationWindow) {


    QObject* scanLeftMenuFrame = applicationWindow->findChild<QObject*>("scanLeftMenuFrame");

    if(scanLeftMenuFrame){
        connect(scanLeftMenuFrame, SIGNAL(startScanning(QString)), parent, SLOT(startScanningSlot(QString)));
        connect(scanLeftMenuFrame, SIGNAL(playPauseScanning(QString, bool, bool)), parent, SLOT(playPauseScanningSlot(QString, bool, bool)));
        connect(scanLeftMenuFrame, SIGNAL(sendTeleop(QString, int)), parent, SLOT(sendTeleopSlot(QString, int)));
        connect(this, SIGNAL(receivedScanMap(QVariant)), scanLeftMenuFrame, SLOT(receivedScanMap(QVariant)));
    }
}

void ScanMapController::receivedScanMap(QString ip, QImage map, QString resolution){
    emit receivedScanMap(ip);
    /// TODO do stuff with the map
}
