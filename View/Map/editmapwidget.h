#ifndef EDITMAPWIDGET_H
#define EDITMAPWIDGET_H

class Map;
class QHBoxLayout;
class CustomQGraphicsView;
class EditMapView;
class CustomLineEdit;
class QButtonGroup;

#include "Model/Map/map.h"
#include <QWidget>
#include <QGraphicsScene>

class EditMapWidget : public QWidget {
    Q_OBJECT
public:
    EditMapWidget(QImage _mapImage, int _width, int _height, float _mapResolution, Position _mapOrigin, QWidget* parent = Q_NULLPTR);
    ~EditMapWidget();

    QImage getMapImage(void) const { return mapImage; }
    float getResolution(void) const { return mapResolution; }
    int getWidth(void) const { return mapWidth; }
    int getHeight(void) const { return mapHeight; }
    Position getOrigin(void) const { return mapOrigin; }

protected:
    void initializeMenu();
    void initializeMap();
    void centerMap();

private slots:
    void cancelSlot();
    void saveSlot();
    void changeSizeEditSlot();
    void changeLineEditSlot(int);

signals:
    void changeSizeEdit(int size);
    void saveEditMap();

private:
    QHBoxLayout* layout;
    QImage mapImage;
    int mapWidth;
    int mapHeight;
    float mapResolution;
    Position mapOrigin;

    QGraphicsScene* scene;
    CustomQGraphicsView* graphicsView;
    QGraphicsPixmapItem* pixmapItem;
    EditMapView* canvas;
    CustomLineEdit* sizeLineEdit;
    QButtonGroup* sizeGroup;
};

#endif // EDITMAPWIDGET_H
