#ifndef MAPCONTROLLER_H
#define MAPCONTROLLER_H

#include <QObject>
#include "Model/Map/map.h"

class MapController : public QObject {

    Q_OBJECT

public:
    MapController(QObject *applicationWindow, QObject* parent);

    QString getMapFile(void) const { return map->getMapFile(); }
    QImage getMapImage(void) const { return map->getMapImage(); }

    void initializeMap(void);

    void setMapFile(const QString file) { map->setMapFile(file); }

    bool saveMapConfig(const std::string fileName, const double centerX, const double centerY, const double zoom) const;

private slots:
    void loadStateSlot();

public slots:
    void saveStateSlot(double posX, double posY, double zoom, QString mapSrc);

signals:
    void setMap(QVariant mapSrc);
    void setMapState(QVariant posX, QVariant posY, QVariant zoom);

private:
    Map* map;
};

#endif /// MAPCONTROLLER_H
