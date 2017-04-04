#ifndef MERGEMAPCONTROLLER_H
#define MERGEMAPCONTROLLER_H

#include <QObject>
#include <QVector>
#include <QSize>
#include <QImage>

class QQmlApplicationEngine;
class MergeMapsPaintedItem;
class MainController;

class MergeMapController : public QObject {

    Q_OBJECT

public:

    MergeMapController(MainController* parent, QQmlApplicationEngine *engine, QObject *applicationWindow);

private slots:

    void importMap(const QString &_filename);
    void importMap(QImage image, double _resolution);
    void exportMap(QString fileName);
    void rotateMap(int angle, int index);
    void removeMap(int index);
    void resetMergeMapWidget();

signals:
    void differentMapSizes();

private:
    QQmlApplicationEngine* _engine;
    QObject* _applicationWindow;
    QVector<MergeMapsPaintedItem*> paintedItems;

    double resolution;
    /// updated when the first image is imported to set a reference for the subsequent images
    QSize size_of_images_merged;
};

#endif /// MERGEMAPCONTROLLER_H
