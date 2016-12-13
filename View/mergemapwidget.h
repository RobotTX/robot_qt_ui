#ifndef MERGEMAPWIDGET_H
#define MERGEMAPWIDGET_H

class QHBoxLayout;
class CustomQGraphicsView;
class QButtonGroup;
class MergeMapListWidget;
class MergeMapListItemWidget;


#include <QWidget>
#include <QGraphicsScene>
#include "Model/robots.h"
#include "Model/position.h"

#define MERGE_WIDGET_HEIGHT 120

class MergeMapWidget : public QWidget {
    Q_OBJECT
public:
    MergeMapWidget(QSharedPointer<Robots> _robots, QWidget* parent = Q_NULLPTR);

protected:
    void initializeMenu();
    void initializeMap();
    void refreshIds();
    QImage sceneToImage();
    QImage croppedImageToMapImage(QImage croppedImage);
    bool checkImageSize(QSize sizeCropped);
    double getResolution();

private slots:
    void cancelSlot();
    void saveSlot();
    void resetSlot();
    void addImageFileSlot();
    void addImageRobotSlot();
    void deleteMapSlot(int itemId);
    void dirKeyEventSlot(int key);
    void selectPixmap(int id);
    void robotMenuSlot(QAction*);

signals:
    void saveMergeMap(double, Position, QImage, QString);
    void getMapForMerging(QString);

private:
    MergeMapListWidget* listWidget;
    QHBoxLayout* layout;
    QGraphicsScene* scene;
    CustomQGraphicsView* graphicsView;
    QButtonGroup* sizeGroup;
    QSize originalSize;
    double resolution;
    QPoint croppedOriginInPixel;
    QPoint originInPixel;
    QSharedPointer<Robots> robots;
};

#endif // MERGEMAPWIDGET_H
