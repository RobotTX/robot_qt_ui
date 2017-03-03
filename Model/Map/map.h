#ifndef MAP_H
#define MAP_H

#include <QObject>

/**
 * @brief The Map class
 * The model class for the Map,
 * contains the map as a QImage and its width, height, resolution and the origin
 */
class Map : public QObject {

    Q_OBJECT

public:
    Map(QObject* parent = Q_NULLPTR);


private:

};

#endif /// MAP_H
