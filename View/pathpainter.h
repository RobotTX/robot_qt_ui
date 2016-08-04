#ifndef PATHPAINTER_H
#define PATHPAINTER_H

class MapView;
class CurrentPath;
class Points;
class MainWindow;

#include <QObject>
#include <QGraphicsPathItem>
#include <QPainterPath>
#include <QPen>
#include <QVector>
#include <memory>
#include "Model/point.h"

/**
 * @brief The PathPainter class
 * The class that draw the lines between each points showing the path of a robot
 */
class PathPainter : public QObject, public QGraphicsPathItem{
    Q_OBJECT

public:
    PathPainter(MainWindow* const &mainWindow, MapView* const &mapPixmapItem, std::shared_ptr<Points> points);
    std::shared_ptr<CurrentPath> getCurrentPath(void){ return currentPath; }
    void reset(void);



/*    void updatePath(const QVector<Point>& pointVector, bool save = false);
    void updatePath(const QVector<PointView*>& pointViewsVector, bool save = false);
    void reset(bool save = false);
    void refresh(bool save = false);
    QVector<Point> getPathVector(void) const {return pathVector;}
    void setPointViewPixmap(const int id, PointView* const pointView);
    void clearPointViews(bool save = false);
    void setPathVector(const QVector<Point> _pathvector) { pathVector = _pathvector; }
*/
private slots:
    void addPathPointSlot(QString name, double x, double y);

private:
    QPainterPath path;
    std::shared_ptr<CurrentPath> currentPath;
};

#endif // PATHPAINTER_H
