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
        connect(scanLeftMenuFrame, SIGNAL(rotateMap(int, QString)), this, SLOT(rotateMap(int, QString)));
        connect(scanLeftMenuFrame, SIGNAL(resetScanMaps()), this, SLOT(resetScanMaps()));
        connect(scanLeftMenuFrame, SIGNAL(saveScan(QString)), this, SLOT(saveScanSlot(QString)));

        connect(this, SIGNAL(receivedScanMap(QVariant)), scanLeftMenuFrame, SLOT(receivedScanMap(QVariant)));
    }

    QObject* scanWindow = applicationWindow->findChild<QObject*>("scanWindow");
    if(scanWindow){
        connect(this, SIGNAL(readyToBeGrabbed(QVariant)), scanWindow, SLOT(grabScannedMap(QVariant)));
    }

    QObject* scanMap = applicationWindow->findChild<QObject*>("scanMapView");
    if(scanMap)
        connect(this, SIGNAL(updateSize(QVariant, QVariant)), scanMap, SLOT(adjustSize(QVariant, QVariant)));
    else
        Q_UNREACHABLE();

    QObject* topView = applicationWindow->findChild<QObject*>("topViewFrame");
    if(topView){
        connect(this, SIGNAL(invalidGoal()), topView, SLOT(displayInvalidGoalError()));
    }

    connect(this, SIGNAL(sendGoal(QString,double, double)), parent, SLOT(sendScanGoal(QString, double, double)));
}

void ScanMapController::receivedScanMap(QString ip, QImage map, QString resolution){

    emit receivedScanMap(ip);

    if(!paintedItems.contains(ip)){

        emit updateSize(map.width(), map.height());

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
        paintedItem->setPosition(QPointF(mapView->width()/2, mapView->height()/2));

        paintedItem->setProperty("ip", ip);
        paintedItem->setProperty("width", paintedItem->getImage().width());
        paintedItem->setProperty("height", paintedItem->getImage().height());
        /// at first we draw the robot on the map, we will hide it just before saving
        paintedItem->setProperty("_drawRobotView", true);
        paintedItem->update();

        qDebug() << "inserting ip" << ip;
        colors.insert(ip, paintedItems.count());
        paintedItems.insert(ip, paintedItem);

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

void ScanMapController::removeMap(QString ip){
    qDebug() << "ScanMapController::removeMap called with ip" << ip;
    if(paintedItems.contains(ip)){
        paintedItems[ip]->setVisible(false);
        paintedItems.remove(ip);
    }
}

void ScanMapController::rotateMap(int angle, QString ip){
    if(paintedItems.contains(ip))
        paintedItems[ip]->rotate(angle);
}

void ScanMapController::resetScanMaps(){
    qDebug() << "ScanMapController::resetScanMaps called";
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

    QMapIterator<QString, ScanMapPaintedItem*> it(paintedItems);
    while(it.hasNext()){
        it.next();
        QImage& image = it.value()->getImage();
        for(int i = 0; i < image.width(); i++){
            for(int j = 0; j < image.height(); j++){
                QColor color = image.pixelColor(i, j);
                if(!(color.red() == 205 && color.green() == 205 && color.blue() == 205)){
                    /// If we find a pixel with more blue than the rest, it is our origin pixel
                    if(color.red() == color.green() && color.blue() > color.red()){
                        image.setPixelColor(i, j, Qt::white);
                    } else if(color.red() == color.green() && color.green() == color.blue())
                        image.setPixelColor(i, j, Qt::white);
                    else
                        image.setPixelColor(i, j, Qt::black);
                }
            }
        }
        it.value()->setProperty("_drawRobotView", false);
        it.value()->update();    
    }

    emit readyToBeGrabbed(file_name);
}

void ScanMapController::sendGoalSlot(QString ip, double x, double y){
    qDebug() << "ScanMapController::sendGoalSlot called" << ip << x + paintedItems[ip]->getLeft() << y + paintedItems[ip]->getTop();
    if(paintedItems.contains(ip)){
        /// it is an empty area of the map
        if(paintedItems[ip]->getImage().pixelColor(x, y).red() > 250)
            emit sendGoal(ip, x + paintedItems[ip]->getLeft(), y + paintedItems[ip]->getTop());
        else
            emit invalidGoal();
    }
}
