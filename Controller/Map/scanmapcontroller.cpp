#include "scanmapcontroller.h"
#include "Controller/maincontroller.h"
#include "Helper/helper.h"
#include <QQmlApplicationEngine>

ScanMapController::ScanMapController(MainController* parent, QQmlApplicationEngine* _engine, QObject *_applicationWindow)
    : QObject(parent), engine(_engine), applicationWindow(_applicationWindow) {

    QObject* scanLeftMenuFrame = applicationWindow->findChild<QObject*>("scanLeftMenuFrame");

    if(scanLeftMenuFrame){
        connect(scanLeftMenuFrame, SIGNAL(startScanning(QString)), parent, SLOT(startScanningSlot(QString)));
        connect(scanLeftMenuFrame, SIGNAL(startAutomaticScan(QString)), parent, SLOT(startAutomaticScanSlot(QString)));
        connect(scanLeftMenuFrame, SIGNAL(playPauseScanning(QString, bool, bool)), parent, SLOT(playPauseScanningSlot(QString, bool, bool)));
        connect(scanLeftMenuFrame, SIGNAL(playPauseExploring(QString, bool)), parent, SLOT(playPauseExploringSlot(QString, bool)));
        connect(scanLeftMenuFrame, SIGNAL(sendTeleop(QString, int)), parent, SLOT(sendTeleopSlot(QString, int)));
        connect(scanLeftMenuFrame, SIGNAL(resetScanMaps()), this, SLOT(resetScanMaps()));
        connect(scanLeftMenuFrame, SIGNAL(saveScan(QString)), this, SLOT(saveScanSlot(QString)));
        connect(scanLeftMenuFrame, SIGNAL(removeMap(QString)), this, SLOT(removeMap(QString)));
        connect(scanLeftMenuFrame, SIGNAL(requestCoordinatesRobotAndScanMapItem(QString)), SLOT(sendCoordinatesRobotAndScanMapItem(QString)));

        connect(this, SIGNAL(receivedScanMap(QVariant)), scanLeftMenuFrame, SLOT(receivedScanMap(QVariant)));
    }

    QObject* scanWindow = applicationWindow->findChild<QObject*>("scanWindow");
    if(scanWindow){
        connect(this, SIGNAL(readyToBeGrabbed(QVariant)), scanWindow, SLOT(grabScannedMap(QVariant)));
        connect(this, SIGNAL(coordinatesRobotAndScanMapItem(QVariant,QVariant, QVariant, QVariant)), scanWindow, SLOT(centerOnRobot(QVariant, QVariant, QVariant, QVariant)));
        connect(scanWindow, SIGNAL(resetMapConfiguration(QString, bool)), parent, SLOT(resetMapConfiguration(QString, bool)));
        connect(scanWindow, SIGNAL(discardMap(bool)), parent, SLOT(setDiscardMap(bool)));
    }

    QObject* scanMap = applicationWindow->findChild<QObject*>("scanMapView");
    if(scanMap){
        connect(this, SIGNAL(updateSize(QVariant, QVariant)), scanMap, SLOT(adjustSize(QVariant, QVariant)));
    } else
        /// NOTE can probably remove that when testing phase is over
        Q_UNREACHABLE();

    connect(this, SIGNAL(sendGoal(QString, double, double)), parent, SLOT(sendScanGoal(QString, double, double)));
    connect(this, SIGNAL(setMessageTop(int, QString)), parent, SLOT(setMessageTopSlot(int, QString)));
    connect(this, SIGNAL(clearPointsAndPaths()), parent, SLOT(clearPointsAndPathsAfterScan()));
    connect(this, SIGNAL(discardMap(bool)), parent, SLOT(setDiscardMap(bool)));
}

void ScanMapController::receivedScanMap(QString ip, QImage map, QString resolution){

    /// we set mapreceived property to true
    emit receivedScanMap(ip);

    if(!paintedItems.contains(ip)){

        emit updateSize(map.width(), map.height());
//        // qDebug() << "map.width = " << map.width() << " map.height = " << map.height();

        QQmlComponent component(engine, QUrl("qrc:/View/ScanMap/ScanMapsPaintedItem.qml"));
        ScanMapPaintedItem* paintedItem = qobject_cast<ScanMapPaintedItem*>(component.create());
        QQmlEngine::setObjectOwnership(paintedItem, QQmlEngine::CppOwnership);

        connect(paintedItem, SIGNAL(sendGoal(QString, double, double)), this, SLOT(sendGoalSlot(QString, double, double)));

        /// that is where we actually tell the paintemItem to paint itself
        QQuickItem* mapView = applicationWindow->findChild<QQuickItem*> ("scanMapView");

        paintedItem->setParentItem(mapView);
        paintedItem->setParent(engine);

        /// we crop before drawing
        paintedItem->setImage(Helper::Image::crop(map, paintedItems.count()));
        paintedItem->setPosition(QPointF(mapView->width()/2 - paintedItem->getImage().width()/2,
                                         mapView->height()/2 - paintedItem->getImage().height()/2));

        paintedItem->setProperty("ip", ip);

        QVariant name;
        QMetaObject::invokeMethod(applicationWindow->findChild<QObject*>("robotModel"), "getName",
                Q_RETURN_ARG(QVariant, name),
                Q_ARG(QVariant, ip));
//        // qDebug() << "Mah name" << name;

        paintedItem->setProperty("name", name.toString());
        paintedItem->setProperty("width", paintedItem->getImage().width());
        paintedItem->setProperty("height", paintedItem->getImage().height());
        paintedItem->update();

        colors.insert(ip, paintedItems.count());
        paintedItems.insert(ip, paintedItem);

    } else {
        paintedItems[ip]->setImage(Helper::Image::crop(map, colors[ip]));
        paintedItems[ip]->setProperty("width", paintedItems[ip]->getImage().width());
        paintedItems[ip]->setProperty("height", paintedItems[ip]->getImage().height());
        paintedItems[ip]->update();
    }
}

void ScanMapController::updateRobotPos(QString ip, double x, double y, double orientation){
    if(paintedItems.contains(ip)){
        paintedItems[ip]->setRobotX(x);
        paintedItems[ip]->setRobotY(y);
        paintedItems[ip]->setRobotOrientation(orientation);
    }
}

void ScanMapController::removeMap(QString ip){
//    // qDebug() << "ScanMapController::removeMap called with ip" << ip;
    if(paintedItems.contains(ip)){
        paintedItems[ip]->setVisible(false);
        paintedItems.remove(ip);
    }
}

void ScanMapController::resetScanMaps(){
//    // qDebug() << "ScanMapController::resetScanMaps called";
    QMapIterator<QString, ScanMapPaintedItem*> it(paintedItems);
    while(it.hasNext()){
        it.next();
        it.value()->setVisible(false);
    }
    paintedItems.clear();
}

void ScanMapController::saveScanSlot(QString file_name){
    /// We want to find the smallest rectangle containing the map so we can find its center and put it at the center of the window
    /// before grab
    emit discardMap(true);

//    // qDebug() << "paintedItems.size = " << paintedItems.size();

    if(paintedItems.size() > 0){
        QMapIterator<QString, ScanMapPaintedItem*> it(paintedItems);
        while(it.hasNext()){
            it.next();
            QImage& image = it.value()->getImage();
//            // qDebug() << "image.width = " << image.width() << "image.height = " << image.height();
            for(int i = 0; i < image.width(); i++){
                for(int j = 0; j < image.height(); j++){
                    QColor color = image.pixelColor(i, j);
                    if(!(color.red() == 205 && color.green() == 205 && color.blue() == 205)){
                        if(color.red() == color.green() && color.green() == color.blue())
                            image.setPixelColor(i, j, Qt::white);
                        else
                            image.setPixelColor(i, j, Qt::black);
                    }
                }
            }
            it.value()->hideRobot();
            it.value()->update();
        }

        /// notify qml that map ready to be saved in file_name
        qDebug() << "Filename in scanmapcontroller for readyToBeGrabbed : " << file_name;
        qDebug() << "In saveScanSlot, we are going to call the following functions : " << endl;
        qDebug() << "readyToBeGrabbed" << endl;
        qDebug() << "clearPointsAndPaths" << endl;
        emit readyToBeGrabbed(file_name);
        emit setMessageTop(2, "Finished to scan the new map");
        emit clearPointsAndPaths();
    }
}

void ScanMapController::sendGoalSlot(QString ip, double x, double y){
//    // qDebug() << "ScanMapController::sendGoalSlot called" << ip << x + paintedItems[ip]->getLeft() << y + paintedItems[ip]->getTop();
    if(paintedItems.contains(ip)){
        /// it is an empty area of the map
        if(paintedItems[ip]->getImage().pixelColor(x, y).red() > 250)
            emit sendGoal(ip, x + paintedItems[ip]->getLeft(), y + paintedItems[ip]->getTop());
        else
            emit invalidGoal();
    }
}

void ScanMapController::sendCoordinatesRobotAndScanMapItem(QString ip){
    if(paintedItems.contains(ip)){
        ScanMapPaintedItem* item = paintedItems.value(ip);
//        // qDebug() << "To center scan map sending robot and scan map pos" <<
//                    item->robotX() << item->robotY() << item->x() << item->y();
        emit coordinatesRobotAndScanMapItem(item->robotX(), item->robotY(), item->x(), item->y());
    } else
        Q_UNREACHABLE();
}
