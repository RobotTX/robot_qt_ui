#ifndef EDITMAPWIDGET_H
#define EDITMAPWIDGET_H

class Map;
class QHBoxLayout;
class CustomQGraphicsView;
class EditMapView;
class CustomLineEdit;
class QButtonGroup;

#include <QWidget>
#include <QGraphicsScene>

class EditMapWidget : public QWidget {
    Q_OBJECT
public:
    EditMapWidget(QImage _mapImage, int _width, int _height, QWidget *parent = Q_NULLPTR);
    ~EditMapWidget();

    QImage getImage(void) const { return mapImage; }

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

    QGraphicsScene* scene;
    CustomQGraphicsView* graphicsView;
    QGraphicsPixmapItem* pixmapItem;
    EditMapView* canvas;
    CustomLineEdit* sizeLineEdit;
    QButtonGroup* sizeGroup;
};

#endif // EDITMAPWIDGET_H
