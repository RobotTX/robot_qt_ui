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

private slots:
    void saveMapConfig(double zoom, double centerX, double centerY) const;

private:
    Map* map;
};

#endif /// MAPCONTROLLER_H
