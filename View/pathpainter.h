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

    QVector<QSharedPointer<PathPoint>> getCurrentPath(void) const { return currentPath; }
    QVector<QSharedPointer<PathPoint>> getOldPath(void) const { return oldPath; }

    void setCurrentPath(const QVector<QSharedPointer<PathPoint> > _currentPath);
    void setOldPath(const QVector<QSharedPointer<PathPoint> > _oldPath);
    void displayPath(void);
    int nbUsedPointView(const QString name, const double x, const double y);
    void updateCurrentPath(void);
    void updatePathPainterName(void);
    void clearOldPath();
    bool getPathDeleted(void) const { return pathDeleted; }
    void setPathDeleted(bool _pathDeleted){ pathDeleted = _pathDeleted; }

private slots:
    void resetPathSlot(void);
    void addPathPointSlot(QString name, double x, double y);
    void deletePathPointSlot(int id);
    void updatePathPainterSlot(void);
    void updatePathPainterPointViewSlot(void);
    void orderPathPointChangedSlot(int from, int to);
    void actionChangedSlot(int id, int action, QString waitTime);
    void editPathPointSlot(int id, QString name, double x, double y);

signals:
    void updatePoints(int id, QString name);

private:
    QPainterPath path;
    QSharedPointer<Points> points;
    /// changing as the user edits the path of its robot
    QVector<QSharedPointer<PathPoint>> currentPath;
    QVector<QSharedPointer<PathPoint>> oldPath;
    MainWindow* mainWindow;
    MapView* mapView;
    bool pathDeleted;
};

#endif // PATHPAINTER_H
