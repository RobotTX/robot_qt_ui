#include <QDebug>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickItemGrabResult>
#include <QPixmap>
#include <QQuickItem>
#include <QQuickWindow>
#include <QPainter>
#include "mergemapcontroller.h"
#include "View/mergemapspainteditem.h"
#include "Controller/maincontroller.h"

MergeMapController::MergeMapController(MainController *parent, QQmlApplicationEngine* engine, QObject *applicationWindow)
    : QObject(parent), _engine(engine), _applicationWindow(applicationWindow)
{
    QObject* mergeMapWindow = applicationWindow->findChild<QObject*>("mergeMapWindow");

    if(mergeMapWindow){
        /// to add new maps
        connect(mergeMapWindow, SIGNAL(importMap(QString)), this, SLOT(importMap(QString)));
        connect(mergeMapWindow, SIGNAL(exportMap(QString)), this, SLOT(exportMap(QString)));
        connect(mergeMapWindow, SIGNAL(resetWidget()), this, SLOT(resetMergeMapWidget()));
        connect(mergeMapWindow, SIGNAL(getMapFromRobot(QString)), parent, SLOT(getMapFromRobot(QString)));

        connect(this, SIGNAL(differentMapSizes()), mergeMapWindow, SLOT(cancelImportMap()));
        connect(this, SIGNAL(readyToBeGrabbed(QVariant)), mergeMapWindow, SLOT(grabMergedMap(QVariant)));

    }

    QObject* mergeMapsView = applicationWindow->findChild<QObject*>("mergeMapsView");
/*
    if(mergeMapsView){
        connect(this, SIGNAL(readyToBeGrabbed(QVariant)), mergeMapsView, SLOT(grabMergedMap(QVariant)));
    }*/


    QObject* mergeMapLeftMenu = applicationWindow->findChild<QObject*>("mergeMapLeftMenu");
    if(mergeMapLeftMenu){
        /// to rotate maps
        connect(mergeMapLeftMenu, SIGNAL(rotate(int, int)), this, SLOT(rotateMap(int, int)));
        connect(mergeMapLeftMenu, SIGNAL(removeMap(int)), this, SLOT(removeMap(int)));
    }
}

void MergeMapController::importMap(const QString& _filename){
    qDebug() << "MergeMapController importMap called" << _filename;
    QImage image(_filename, "PGM");
    if(paintedItems.size() == 0)
        size_of_images_merged = image.size();

    if(image.size() == size_of_images_merged){
        QQmlComponent component(_engine, QUrl("qrc:/View/Map/MergeMapsPaintedItem.qml"));
        MergeMapsPaintedItem* paintedItem = qobject_cast<MergeMapsPaintedItem*>(component.create());
        QQmlEngine::setObjectOwnership(paintedItem, QQmlEngine::CppOwnership);

        /// that is where we actually tell the paintemItem to paint itself
        QQuickItem* mapView = _applicationWindow->findChild<QQuickItem*> ("mergeMapsView");
        paintedItem->setParentItem(mapView);
        paintedItem->setParent(_engine);

        qDebug() << "imported map of size" << image.width() << image.height();

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

        qDebug() << "cropping with values" << top << left << bottom << right;

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
    if(paintedItems.isEmpty())
        size_of_images_merged = image.size();

    if(image.size() == size_of_images_merged){
        QQmlComponent component(_engine, QUrl("qrc:/View/Map/MergeMapsPaintedItem.qml"));
        MergeMapsPaintedItem* paintedItem = qobject_cast<MergeMapsPaintedItem*>(component.create());
        QQmlEngine::setObjectOwnership(paintedItem, QQmlEngine::CppOwnership);

        /// that is where we actually tell the paintemItem to paint itself
        QQuickItem* mapView = _applicationWindow->findChild<QQuickItem*> ("mergeMapsView");
        paintedItem->setParentItem(mapView);
        paintedItem->setParent(_engine);

        qDebug() << "imported map of size" << image.width() << image.height();

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

        qDebug() << "cropping with values" << top << left << bottom << right;

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
    qDebug() << "exportMap called" << fileName;
    QQuickItem* mapView = _applicationWindow->findChild<QQuickItem*> ("mergeMapsView");

    qDebug() << "\nMergeMapWidget::saveSlot called";

    QImage final_image(QSize(2048, 2048), QImage::Format_ARGB32);
    final_image.fill(QColor(205, 205, 205));
    QPainter painter(&final_image);

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

    emit readyToBeGrabbed(fileName);

}

void MergeMapController::rotateMap(int angle, int index){
    paintedItems.at(index)->rotate(angle);
}

void MergeMapController::removeMap(int index){
    qDebug() << "MergeMapController::removemap called" << index;
    paintedItems.at(index)->setVisible(false);
    paintedItems.remove(index);
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
/*
QImage MergeMapWidget::sceneToImage(){
    qDebug() << "MergeMapWidget::sceneToImage called";
    /// Center the scene on the images
    scene->clearSelection();
    scene->setSceneRect(scene->itemsBoundingRect());
    /// Create an image with the size the images are taking in the scene
    QImage image(scene->sceneRect().size().toSize(), QImage::Format_ARGB32);
    /// Fill the image with grey
    image.fill(QColor(205, 205, 205));
    /// We use a painter to copy the scene into the image
    QPainter painter(&image);
    scene->render(&painter);
    /// The image is still in green and red color so we set the pixel to white and black
    for(int i = 0; i < image.width(); i++){
        for(int j = 0; j < image.height(); j++){
            QColor color = image.pixelColor(i, j);
            if(!(color.red() == 205 && color.green() == 205 && color.blue() == 205)){
                /// If we find a pixel with more blue than the rest, it is our origin pixel
                if(color.red() == color.green() && color.blue() > color.red()){
                    image.setPixelColor(i, j, Qt::white);
                    croppedOriginInPixel = QPoint(i, j);
                    qDebug() << "Got an origin :" << croppedOriginInPixel;
                } else if(color.red() == color.green() && color.green() == color.blue())
                    image.setPixelColor(i, j, Qt::white);
                else
                    image.setPixelColor(i, j, Qt::black);
            }
        }
    }
    return image;
}

QImage MergeMapWidget::croppedImageToMapImage(QImage croppedImage){
    qDebug() << "MergeMapWidget::croppedImageToMapImage called";
    /// If we don't have an origin, we set the origin in the middle of the map
    if(croppedOriginInPixel.x() == -1){
        croppedOriginInPixel.setX(croppedImage.width()/2);
        croppedOriginInPixel.setY(croppedImage.height()/2);
    }
    /// Create the new image with the desired size
    QImage image(originalSize, QImage::Format_ARGB32);
    /// Fill the image with grey
    image.fill(QColor(205, 205, 205));
    /// Calculate the size difference between the given image and the output image
    int leftDiff = qAbs((originalSize.width() - croppedImage.width())/2);
    int topDiff = qAbs((originalSize.height() - croppedImage.height())/2);
    int leftSign = 1;
    int topSign = 1;
    /// If the input image is bigger on both sides, we crop the image
    if(croppedImage.width() > originalSize.width() && croppedImage.height() > originalSize.height()){
        qDebug() << "MergeMapWidget::croppedImageToMapImage width & height are bigger";
        QImage tmpImage = croppedImage.copy(leftDiff, topDiff, originalSize.width(), originalSize.height());
        QPainter painter(&image);
        painter.drawImage(0, 0, tmpImage);
        painter.end();
        leftSign = -1;
        topSign = -1;
    } else if(croppedImage.width() > originalSize.width()){
        qDebug() << "MergeMapWidget::croppedImageToMapImage width is bigger";
        QImage tmpImage = croppedImage.copy(leftDiff, 0, originalSize.width(), originalSize.height());
        QPainter painter(&image);
        painter.drawImage(0, 0, tmpImage);
        painter.end();
        leftSign = -1;
    } else if(croppedImage.height() > originalSize.height()){
        qDebug() << "MergeMapWidget::croppedImageToMapImage height is bigger";
        QImage tmpImage = croppedImage.copy(0, topDiff, originalSize.width(), originalSize.height());
        QPainter painter(&image);
        painter.drawImage(0, 0, tmpImage);
        painter.end();
        topSign = -1;
    } else {
        qDebug() << "MergeMapWidget::croppedImageToMapImage everything is fine";
        for(int i = 0; i < croppedImage.width(); i++)
            for(int j = 0; j < croppedImage.height(); j++)
                image.setPixelColor(i + leftDiff, j + topDiff, croppedImage.pixelColor(i, j));
    }
    /// Calculate the position of the origin in the output image
    originInPixel.setX(croppedOriginInPixel.x() + leftDiff * leftSign);
    originInPixel.setY(croppedOriginInPixel.y() + topDiff * topSign);
    /// If the origin is out of the map (because we cropped it), we set it in the middle of the map
    if(originInPixel.x() < 0 || originInPixel.y() < 0 || originInPixel.x() >= image.width() || originInPixel.y() >= image.height()){
        originInPixel.setX(image.width()/2);
        originInPixel.setY(image.height()/2);
    }
    return image;
}
*/
