#ifndef MAPCONTROLLER_H
#define MAPCONTROLLER_H

class Map;

#include <QObject>

class MapController : public QObject {
    Q_OBJECT
public:
    MapController(QObject *applicationWindow, QObject* parent);

private:
        Map* map;
};

#endif // MAPCONTROLLER_H
