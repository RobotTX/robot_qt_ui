#ifndef EDITMAPWIDGET_H
#define EDITMAPWIDGET_H

class Map;
class QHBoxLayout;
class CustomQGraphicsView;
class EditMapView;

#include <QWidget>
#include <QGraphicsScene>

class EditMapWidget : public QWidget {
    Q_OBJECT
public:
    EditMapWidget(QImage _mapImage, int _width, int _height, QWidget *parent = Q_NULLPTR);

protected:
    void initializeMenu();
    void initializeMap();
    void centerMap();

private slots:
    void cancelSlot();
    void saveSlot();
    void undoSlot();
    void redoSlot();
    void changeColorSlot(int);
    void changeShapeSlot(int);

private:
    QHBoxLayout* layout;

    QImage mapImage;
    int mapWidth;
    int mapHeight;

    QGraphicsScene* scene;
    CustomQGraphicsView* graphicsView;
    EditMapView* pixmapItem;
};

#endif // EDITMAPWIDGET_H
