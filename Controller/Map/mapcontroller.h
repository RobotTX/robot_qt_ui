#ifndef MAPCONTROLLER_H
#define MAPCONTROLLER_H

#include <QObject>
#include "View/Map/mapview.h"
#include <QSharedPointer>
#include "Model/Other/graphicitemstate.h"
#include "Model/Map/map.h"
#include <QGraphicsScene>
#include "View/Map/customqgraphicsview.h"

class DrawObstacles;
class Robots;
class PointView;
class MainWindow;

class MapController: public QObject
{
    Q_OBJECT

public:

    MapController(QSharedPointer<Robots> _robots, MainWindow *mainWindow);

    QSharedPointer<Map> getMap(void) const { return map; }
    MapView* getMapView(void) const { return view; }
    CustomQGraphicsView* getGraphicsView(void) const { return graphicsView; }
    DrawObstacles* getObstaclesPainter(void) const { return obstaclesPainter; }

    void setTmpPointView(QSharedPointer<PointView> pv) { view->setTmpPointView(pv); }
    void createMapView(MainWindow *mainWindow, QSharedPointer<Robots> _robots);

    QImage getMapImage(void) const { return map->getMapImage(); }
    QPair<QPointF, float> getMapState(void) const { return map->getMapState(); }
    int getMapWidth(void) const { return map->getWidth(); }
    int getMapHeight(void) const { return map->getHeight(); }
    float getMapResolution(void) const { return map->getResolution(); }
    Position getMapOrigin(void) const { return map->getOrigin(); }
    QUuid getMapId(void) const { return map->getMapId(); }
    QDateTime getMapTime(void) const { return map->getDateTime(); }
    std::string getMapFile(void) const { return map->getMapFile(); }
    float getZoomCoeff(void) const { return graphicsView->getZoomCoeff(); }
    bool hasMapChanged(void) const { return map->getModified(); }

    void setMapPosition(const QPointF& pos) { view->setPos(pos); }
    void setZoomCoeff(const float coeff) { graphicsView->setZoomCoeff(coeff); }
    void modifyMap(void) { view->setPixmap(QPixmap::fromImage(map->getMapImage())); }
    void modifyMap(const QPixmap& pixmap) { view->setPixmap(pixmap); }

    void setMapState(const GraphicItemState state);
    void updateMap(void);
    void modifyMap(const QByteArray, const bool fromPgm, const QString id, const QString date);

    QColor getPixelColor(const double x, const double y);

    QImage getImageFromArray(const QByteArray array, const bool fromPgm);
    QImage getImageFromArray(const QByteArray array, int map_width, int map_height, const bool fromPgm);


    bool loadMapConfig(const std::string file);

    void updateMapFile(const std::string file) { map->setMapFile(file); }
    void saveMapToFile(const QString file) { map->saveToFile(file); }

    void moveMap(const QPointF& pos) { view->setPos(pos); }

    void updateMap(const std::string mapFile, const float resolution, const int width, const int height, const Position& origin, const QImage &image, const QUuid id, const QDateTime date);
    void updateMap(const std::string mapFile, const float resolution, const int width, const int height, const Position& origin, const QPixmap &pix, const QUuid id, const QDateTime date);

    void updateScene(void) { scene->update(); }

    void showGraphicsView(void) const { graphicsView->show(); }

    void removeFromScene(QGraphicsItem* item) { scene->removeItem(item); }

private slots:
    void relayPathPoint(QString, double, double);
    void updateMetadata(const int width, const int height, const float resolution,
    const float originX, const float originY);

public slots:
    void saveMapState();
    bool saveMapConfig(const std::string fileName);

signals:
    void pathPointSignal(QString, double, double);
    void newMessage(QString);

private:
    QGraphicsScene* scene;
    CustomQGraphicsView* graphicsView;
    QSharedPointer<Map> map;
    MapView* view;
    DrawObstacles* obstaclesPainter;
};

#endif /// MAPCONTROLLER_H
