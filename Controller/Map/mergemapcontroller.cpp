#include <QDebug>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickItemGrabResult>
#include <QPixmap>
#include <QQuickItem>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QQuickWindow>
#include <QPainter>
#include "mergemapcontroller.h"
#include "View/MergeMap/mergemapspainteditem.h"
#include "Controller/maincontroller.h"

MergeMapController::MergeMapController(MainController *parent, QQmlApplicationEngine* _engine, QObject *_applicationWindow)
    : QObject(parent), engine(_engine), applicationWindow(_applicationWindow) {

    QObject* mergeMapWindow = applicationWindow->findChild<QObject*>("mergeMapWindow");

    if(mergeMapWindow){
        /// to add new maps
        connect(mergeMapWindow, SIGNAL(resetWidget()), this, SLOT(resetMergeMapWidget()));
        connect(mergeMapWindow, SIGNAL(resetMapConfiguration(QString, bool)), parent, SLOT(resetMapConfiguration(QString, bool)));
        connect(this, SIGNAL(differentMapSizes()), mergeMapWindow, SLOT(cancelImportMap()));
        connect(this, SIGNAL(readyToBeGrabbed(QVariant)), mergeMapWindow, SLOT(grabMergedMap(QVariant)));

    } else
        Q_UNREACHABLE();

    QObject* mergeMap = applicationWindow->findChild<QObject*>("mergeMapsView");
    if(mergeMap)
        connect(this, SIGNAL(updateSize(QVariant, QVariant)), mergeMap, SLOT(adjustSize(QVariant, QVariant)));
    else
        /// NOTE can probably remove that when testing phase is over
        Q_UNREACHABLE();

    QObject* mergeMapLeftMenu = applicationWindow->findChild<QObject*>("mergeMapLeftMenu");
    if(mergeMapLeftMenu){
        connect(mergeMapLeftMenu, SIGNAL(getMapFromRobot(QString)), parent, SLOT(getMapFromRobot(QString)));
        connect(mergeMapLeftMenu, SIGNAL(importMap(QString)), this, SLOT(importMap(QString)));
        connect(mergeMapLeftMenu, SIGNAL(exportMap(QString)), this, SLOT(exportMap(QString)));
        /// to rotate maps
        connect(mergeMapLeftMenu, SIGNAL(rotate(int, int)), this, SLOT(rotateMap(int, int)));
        connect(mergeMapLeftMenu, SIGNAL(removeMap(int)), this, SLOT(removeMap(int)));
    } else
        Q_UNREACHABLE();

    connect(this, SIGNAL(setMessageTop(int, QString)), parent, SLOT(setMessageTopSlot(int, QString)));
}

void MergeMapController::importMap(const QString& _filename){
    qDebug() << "MergeMapController importMap called" << _filename;
    QImage image(_filename, "PGM");
    if(paintedItems.size() == 0){
        emit updateSize(image.width(), image.height());
        size_of_images_merged = image.size();
    }

    if(image.size() == size_of_images_merged){
        QQmlComponent component(engine, QUrl("qrc:/View/MergeMap/MergeMapsPaintedItem.qml"));
        MergeMapsPaintedItem* paintedItem = qobject_cast<MergeMapsPaintedItem*>(component.create());
        QQmlEngine::setObjectOwnership(paintedItem, QQmlEngine::CppOwnership);

        /// that is where we actually tell the paintemItem to paint itself in the merge map view
        QQuickItem* mapView = applicationWindow->findChild<QQuickItem*> ("mergeMapsView");
        paintedItem->setParentItem(mapView);
        paintedItem->setParent(engine);

        int top = 0;
        int bottom = image.height();
        int left = image.width();
        int right = 0;

        /// We want to find the smallest rectangle containing the map (white and black) to crop it and use a small image

        for(int i = 0; i < image.width(); i++){
            for(int j = 0; j < image.height(); j++){
                int color = image.pixelColor(i, j).red();
                if(color == 255 || color == 0){
                    if(bottom > j)
                        bottom = j;
                    if(top < j)
                        top = j;
                    if(left > i)
                        left = i;
                    if(right < i)
                        right = i;
                }
            }
        }

        /// We crop the image
        image = image.copy(QRect(QPoint(left, bottom), QPoint(right, top)));

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
        paintedItem->setPosition(QPointF(mapView->width()/2, mapView->height()/2));
        paintedItem->update();
        /// adds the image to the list of the image drawn
        paintedItems.append(paintedItem);
    } else
        emit differentMapSizes();
}

void MergeMapController::importMap(QImage image, double _resolution){
    qDebug() << "Import map from robot" << image.size();
    resolution = _resolution;
    if(paintedItems.isEmpty()){
        emit updateSize(image.width(), image.height());
        size_of_images_merged = image.size();
    }

    if(image.size() == size_of_images_merged){
        QQmlComponent component(engine, QUrl("qrc:/View/MergeMap/MergeMapsPaintedItem.qml"));
        MergeMapsPaintedItem* paintedItem = qobject_cast<MergeMapsPaintedItem*>(component.create());
        QQmlEngine::setObjectOwnership(paintedItem, QQmlEngine::CppOwnership);

        /// that is where we actually tell the paintemItem to paint itself on the merge map view
        QQuickItem* mapView = applicationWindow->findChild<QQuickItem*> ("mergeMapsView");
        paintedItem->setParentItem(mapView);
        paintedItem->setParent(engine);

        int top = 0;
        int bottom = image.height();
        int left = image.width();
        int right = 0;

        /// We want to find the smallest rectangle containing the map (white and black) to crop it and use a small image

        for(int i = 0; i < image.width(); i++){
            for(int j = 0; j < image.height(); j++){
                int color = image.pixelColor(i, j).red();
                if(color == 255 || color == 0){
                    if(bottom > j)
                        bottom = j;
                    if(top < j)
                        top = j;
                    if(left > i)
                        left = i;
                    if(right < i)
                        right = i;
                }
            }
        }

        /// We crop the image
        image = image.copy(QRect(QPoint(left, bottom), QPoint(right, top)));

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

    } else
        emit differentMapSizes();
}

void MergeMapController::exportMap(QString fileName){
    qDebug() << "MergeMapController::exportMap called" << paintedItems.size() << fileName;

    /// We want to find the smallest rectangle containing the map so we can find its center and put it at the center of the window
    /// before grab

    for(int i = 0; i < paintedItems.size(); i++){
        QImage& image = paintedItems.at(i)->getImage();
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
        paintedItems.at(i)->update();
    }

    /// notifies the qml side that the image is ready to be grabbed
    emit readyToBeGrabbed(fileName);
    emit setMessageTop(2, "Finished to merge the maps");
}

void MergeMapController::rotateMap(int angle, int index){
    if(paintedItems.size() > index)
        paintedItems.at(index)->rotate(angle);
}

void MergeMapController::removeMap(int index){
    qDebug() << "MergeMapController::removemap called" << index;
    /// it is common to simply hide the item, it will be destroyed once the application is closed
    /// which is enough if you don't have too many objects created by once instance of the application
    if(paintedItems.size() > index){
        paintedItems.at(index)->setVisible(false);
        paintedItems.remove(index);
    }
    /// if that was the last item we reset the reference size
    if(paintedItems.isEmpty())
        size_of_images_merged = QSize();
}

void MergeMapController::resetMergeMapWidget(){
    qDebug() << "MergeMapController::resetWidget called";
    for(int i = 0; i < paintedItems.count(); i++)
        paintedItems.at(i)->setVisible(false);
    paintedItems.clear();
}
