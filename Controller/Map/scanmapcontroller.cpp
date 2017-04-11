#include "scanmapcontroller.h"
#include "Controller/maincontroller.h"
#include "Helper/helper.h"
#include <QQmlApplicationEngine>

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

    if(!paintedItems.contains(ip)){

        QQmlComponent component(engine, QUrl("qrc:/View/ScanMap/ScanMapsPaintedItem.qml"));
        ScanMapPaintedItem* paintedItem = qobject_cast<ScanMapPaintedItem*>(component.create());
        QQmlEngine::setObjectOwnership(paintedItem, QQmlEngine::CppOwnership);

        /// that is where we actually tell the paintemItem to paint itself
        QQuickItem* mapView = applicationWindow->findChild<QQuickItem*> ("scanMapView");

        paintedItem->setParentItem(mapView);
        paintedItem->setParent(engine);

        /// we crop before drawing
        paintedItem->setImage(Helper::Image::crop(map, paintedItems.count()));
        paintedItem->setPosition(QPointF(mapView->width()/2, mapView->height()/2));

        paintedItem->update();
        qDebug() << "inserting ip" << ip;
        colors.insert(ip, paintedItems.count());
        paintedItems.insert(ip, paintedItem);
        paintedItems[ip]->setProperty("width", paintedItems[ip]->getImage().width());
        paintedItems[ip]->setProperty("height", paintedItems[ip]->getImage().height());

    } else {

        qDebug() << "resetting size to" << map.width() << map.height();
        paintedItems[ip]->setImage(Helper::Image::crop(map, colors[ip]));
        paintedItems[ip]->setProperty("width", paintedItems[ip]->getImage().width());
        paintedItems[ip]->setProperty("height", paintedItems[ip]->getImage().height());
        paintedItems[ip]->update();
    }
}

void ScanMapController::updateRobotPos(QString ip, float x, float y, float orientation){
    if(paintedItems.contains(ip)){
        paintedItems[ip]->setRobotX(x);
        paintedItems[ip]->setRobotY(y);
        paintedItems[ip]->setRobotOrientation(orientation);
    }
}
