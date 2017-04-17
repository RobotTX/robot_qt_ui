#ifndef MERGEMAPCONTROLLER_H
#define MERGEMAPCONTROLLER_H

class QQmlApplicationEngine;
class MergeMapsPaintedItem;
class MainController;

#include <QObject>
#include <QVector>
#include <QSize>
#include <QVariant>
#include <QImage>

class MergeMapController : public QObject {

    Q_OBJECT

public:

    MergeMapController(MainController* parent, QQmlApplicationEngine *engine, QObject *applicationWindow);

private slots:

    /**
     * @brief importMap
     * @param _filename
     * imports a new map to merge from the file <_filename>
     */
    void importMap(const QString &_filename);
    /**
     * @brief importMap
     * @param image
     * @param _resolution
     * imports a new map described by the QImage <image>
     * and updates the resolution to <_resolution>
     */
    void importMap(QImage image, double _resolution);
    /**
     * @brief exportMap
     * @param fileName
     * draws the pixels of the right colors (white, grey, black)
     * in order to prepare the save on the qml side
     */
    void exportMap(QString fileName);
    /**
     * @brief rotateMap
     * @param angle
     * @param index
     * rotates the map at index <index> of an angle <angle> given in degrees
     */
    void rotateMap(int angle, int index);
    /**
     * @brief removeMap
     * @param index
     * removes the map at index <index>
     */
    void removeMap(int index);
    /**
     * @brief resetMergeMapWidget
     * clear the window of all the maps accumulated so far
     */
    void resetMergeMapWidget();

signals:
    /**
     * @brief differentMapSizes
     * notifies the qml side that the map being added does not have the same size as the first one which is forbidden
     */
    void differentMapSizes();
    /**
     * @brief readyToBeGrabbed
     * notifies the qml side that it can save the map in the file given
     * as a parameter
     */
    void readyToBeGrabbed(QVariant);
    /**
     * @brief updateSize
     * Sends the size of the map to the merge and scan widgets so that they can save maps
     * of the same size
     */
    void updateSize(QVariant, QVariant);
    void setMessageTop(int status, QString msg);

private:
    QQmlApplicationEngine* engine;
    QObject* applicationWindow;
    QVector<MergeMapsPaintedItem*> paintedItems;

    double resolution;
    /// updated when the first image is imported to set a reference for the subsequent images
    QSize size_of_images_merged;
};

#endif /// MERGEMAPCONTROLLER_H
