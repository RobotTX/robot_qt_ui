#ifndef PATHPAINTER_H
#define PATHPAINTER_H

class MapView;
class Points;
class MainWindow;

#include <QObject>
#include <QGraphicsPathItem>
#include <QPainterPath>
#include <QPen>
#include <QVector>
#include <QSharedPointer>
#include "Model/point.h"
#include "Model/pathpoint.h"

/**
 * @brief The PathPainter class
 * The class that draw the lines between each points showing the path of a robot
 */
class PathPainter : public QObject, public QGraphicsPathItem{
    Q_OBJECT

public:
    PathPainter(MainWindow* const &mainWindow, MapView* const &mapPixmapItem, QSharedPointer<Points> _points);
    void setCurrentPath(QVector<QSharedPointer<PathPoint> > _currentPath);
    QVector<QSharedPointer<PathPoint>> getCurrentPath(void) const { return currentPath; }
    void displayPath(void);
    int nbUsedPointView(QString name, double x, double y);
    void updateCurrentPath(void);
    void updatePathPainterName(void);

private slots:
    void resetPathSlot(void);
    void addPathPointSlot(QString name, double x, double y);
    void deletePathPointSlot(int id);
    void updatePathPainterSlot(void);
    void updatePathPainterPointViewSlot(void);
    void orderPathPointChangedSlot(int from, int to);
    void actionChangedSlot(int id, int action, QString waitTime);
    void editPathPointSlot(int id, QString name, double x, double y);

private:
    QPainterPath path;
    QSharedPointer<Points> points;
    /// changing as the user edits the path of its robot
    QVector<QSharedPointer<PathPoint>> currentPath;
    MainWindow* mainWindow;
    MapView* mapView;
};

#endif // PATHPAINTER_H
