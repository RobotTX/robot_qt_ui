#ifndef PATHPAINTER_H
#define PATHPAINTER_H

class PathPoint;
class MapView;
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
    PathPainter(MainWindow* const &mainWindow, MapView* const &mapPixmapItem, std::shared_ptr<Points> _points);
    void setCurrentPath(const QVector<std::shared_ptr<PathPoint>>& _currentPath);
    QVector<std::shared_ptr<PathPoint>> getCurrentPath(void) const { return currentPath; }
    void displayPath(void);
    int nbUsedPointView(QString name, double x, double y);
    void updateCurrentPath(void);

private slots:
    void resetPathSlot(void);
    void addPathPointSlot(QString name, double x, double y);
    void deletePathPointSlot(int id);
    void updatePathPainterSlot(void);
    void updatePathPainterPointViewSlot(void);
    void orderPathPointChangedSlot(int from, int to);
    void actionChangedSlot(int id, QString waitTime);
    void editPathPointSlot(int id, QString name, double x, double y);

private:
    QPainterPath path;
    std::shared_ptr<Points> points;
    /// changing as the user edits the path of its robot
    QVector<std::shared_ptr<PathPoint>> currentPath;
    MainWindow* mainWindow;
    MapView* mapView;
};

#endif // PATHPAINTER_H
