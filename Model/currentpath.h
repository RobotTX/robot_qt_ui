#ifndef CURRENTPATH_H
#define CURRENTPATH_H

class PathPoint;
class Points;
class MapView;
class MainWindow;

#include <QObject>
#include <memory>

class CurrentPath : public QObject{
public:
    CurrentPath(MainWindow* const &parent, MapView* const &mapView, std::shared_ptr<Points> _points);
    void setPath(const std::shared_ptr<QVector<std::shared_ptr<PathPoint>>>& _path) { path = _path; }
    std::shared_ptr<QVector<std::shared_ptr<PathPoint>>> getPath(void) const { return path; }
    void reset(void);

private:
    std::shared_ptr<QVector<std::shared_ptr<PathPoint>>> path;
    std::shared_ptr<Points> points;

};

#endif // CURRENTPATH_H
