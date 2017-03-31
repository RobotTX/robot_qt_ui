#include <QDebug>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickItemGrabResult>
#include <QPixmap>
#include <QQuickWindow>
#include "mergemapcontroller.h"
#include "View/mergemapspainteditem.h"

MergeMapController::MergeMapController(QObject *parent, QQmlApplicationEngine* engine, QObject *applicationWindow)
    : QObject(parent), _engine(engine), _applicationWindow(applicationWindow)
{
    QObject* mergeMapWindow = applicationWindow->findChild<QObject*>("mergeMapWindow");

    if(mergeMapWindow){
        /// to add new maps
        connect(mergeMapWindow, SIGNAL(importMap(QString)), this, SLOT(importMap(QString)));
        connect(mergeMapWindow, SIGNAL(exportMap(QString)), this, SLOT(exportMap(QString)));
        connect(mergeMapWindow, SIGNAL(resetWidget()), this, SLOT(resetMergeMapWidget()));
    }

    QObject* mergeMapLeftMenu = applicationWindow->findChild<QObject*>("mergeMapLeftMenu");
    if(mergeMapLeftMenu){
        /// to rotate maps
        connect(mergeMapLeftMenu, SIGNAL(rotate(int, int)), this, SLOT(rotateMap(int, int)));
        connect(mergeMapLeftMenu, SIGNAL(removeMap(int)), this, SLOT(removeMap(int)));
    }
}

void MergeMapController::importMap(const QString& _filename){
    qDebug() << "MergeMapController importMap called" << _filename;
    QQmlComponent component(_engine, QUrl("qrc:/View/Map/MergeMapsPaintedItem.qml"));
    MergeMapsPaintedItem* paintedItem = qobject_cast<MergeMapsPaintedItem*>(component.create());
    QQmlEngine::setObjectOwnership(paintedItem, QQmlEngine::CppOwnership);

    /// that is where we actually tell the paintemItem to paint itself
    QQuickItem* mapView = _applicationWindow->findChild<QQuickItem*> ("mergeMapsView");
    paintedItem->setParentItem(mapView);
    paintedItem->setParent(_engine);

    QImage image(_filename, "PGM");
    qDebug() << "imported map of size" << image.width() << image.height();

    int top = image.height();
    int bottom = 0;
    int left = image.width();
    int right = 0;

    /// We want to find the smallest rectangle containing the map (white and black) to crop it and use a small image

    for(int i = 0; i < image.height(); i++){
        for(int j = 0; j < image.width(); j++){
            int color = image.pixelColor(i, j).red();
            if(color == 255 || color == 0){
                if(top > i)
                    top = i;
                if(left > j)
                    left = j;
                if(bottom < i)
                    bottom = i;
                if(right < j)
                    right = j;
            }
        }
    }

    /// We crop the image
    image = image.copy(top, left, bottom - top + 1, right - left + 1);

    /// Create a new image filled with invisible grey
    QImage new_image = QImage(image.size(), QImage::Format_ARGB32);
    new_image.fill(qRgba(205, 205, 205, 0));

    /// 1 out of 2 map will have red wall and the other one green wall to better distinguish them
    QRgb wallColor = (paintedItems.count() % 2 == 0) ? qRgba(255, 0, 0, 170) : qRgba(0, 255, 0, 170);
    for(int i = 0; i < image.width(); i++){
        for(int j = 0; j < image.height(); j++){
            int color = image.pixelColor(i, j).red();
            if(color < 205)
                new_image.setPixel(i, j, wallColor);
            else if(color > 205)
                new_image.setPixel(i, j, qRgba(255, 255, 255, 170));
        }
    }

    /// so that the properties are set properly on the qml side
    paintedItem->setProperty("width", new_image.width());
    paintedItem->setProperty("height", new_image.height());
    paintedItem->setImage(new_image);

    paintedItem->update();
    /// adds the image to the list of the image drawn
    paintedItems.append(paintedItem);
}

void MergeMapController::exportMap(QString fileName){
    qDebug() << "exportMap called" << fileName;
    QQuickItem* mapView = _applicationWindow->findChild<QQuickItem*> ("mergeMapsView");

    mapView->grabToImage()->image().save("/home/joan/try2.pgm", "PGM");
}

void MergeMapController::rotateMap(int angle, int index){
    paintedItems.at(index)->rotate(angle);
}

void MergeMapController::removeMap(int index){
    qDebug() << "MergeMapController::removemap called" << index;
    paintedItems.at(index)->setVisible(false);
    paintedItems.remove(index);
}

void MergeMapController::resetMergeMapWidget(){
    qDebug() << "MergeMapController::resetWidget called";
    for(int i = 0; i < paintedItems.count(); i++)
        paintedItems.at(i)->setVisible(false);
    paintedItems.clear();
}
