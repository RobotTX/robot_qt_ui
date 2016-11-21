#ifndef EDITMAPWIDGET_H
#define EDITMAPWIDGET_H

class Map;
class QHBoxLayout;
class CustomQGraphicsView;
class EditMapView;
class CustomLineEdit;

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
    void changeSizeSlot(int);
    void changeSizeEditSlot();
    void drawSlot(int, int);

private:
    QHBoxLayout* layout;

    QImage mapImage;
    int mapWidth;
    int mapHeight;

    QGraphicsScene* scene;
    CustomQGraphicsView* graphicsView;
    QGraphicsPixmapItem* pixmapItem;
    EditMapView* canvas;
    CustomLineEdit* sizeLineEdit;
};

#endif // EDITMAPWIDGET_H
