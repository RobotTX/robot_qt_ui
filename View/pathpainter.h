#ifndef PATHPAINTER_H
#define PATHPAINTER_H

class MapView;
class PointsView;
class PointView;

#include <QGraphicsPathItem>
#include <QPainterPath>
#include <QPen>
#include <QVector>
#include "Model/point.h"

/**
 * @brief The PathPainter class
 * The class that draw the lines between each points showing the path of a robot
 */
class PathPainter : public QGraphicsPathItem{

public:
    PathPainter(MapView* const& mapPixmapItem, PointsView* const& _pointViews);
    ~PathPainter();

    void updatePath(QVector<Point> pointVector);
    void reset(void);
    void refresh(void);
    QVector<Point> getPathVector(void) const {return pathVector;}
    void setPointViewPixmap(const int id, PointView* const pointView);
    void clearPointViews(void);

private:
    QPainterPath path;
    QVector<Point> pathVector;
    PointsView* pointViews;
};

#endif // PATHPAINTER_H
