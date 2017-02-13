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
#include "Model/Points/point.h"
#include "Model/Paths/pathpoint.h"

/**
 * @brief The PathPainter class
 * The class that draw the lines between each points showing the path of a robot
 */
class PathPainter : public QObject, public QGraphicsPathItem{
    Q_OBJECT

public:
    PathPainter(MainWindow* const &mainWindow, const QSharedPointer<Points> _points);

    QVector<QSharedPointer<PathPoint>> getCurrentPath(void) const { return currentPath; }
    QVector<QSharedPointer<PathPoint>> getOldPath(void) const { return oldPath; }
    bool getPathDeleted(void) const { return pathDeleted; }
    QString getVisiblePath(void) const { return visiblePath; }

    void setCurrentPath(const QVector<QSharedPointer<PathPoint> > &_currentPath, QString pathName);
    void setOldPath(const QVector<QSharedPointer<PathPoint> > _oldPath);
    void setPathDeleted(const bool _pathDeleted){ pathDeleted = _pathDeleted; }
    void setVisiblePath(const QString path) { visiblePath = path; }

    /**
     * @brief nbUsedPointView
     * @param name
     * @param x
     * @param y
     * @return the number of time the point at the given position (x, y) is used
     */
    int nbUsedPointView(const QString name, const double x, const double y);

    /**
     * @brief updateCurrentPath
     * Update the current path with the points in the group PATH_GROUP_NAME
     */
    void updateCurrentPath(void);

    /**
     * @brief updatePathPainterName
     * Update the name of the points in the current path
     */
    void updatePathPainterName(void);

    /**
     * @brief clearOldPath
     * Delete the content of the old path
     */
    void clearOldPath();

    void resetAllPixmap();

public slots:
    /**
     * @brief updatePathPainterSlot
     * @param savePath
     * Redraw the whole path
     */
    void updatePathPainterSlot(const bool savePath);

    /**
     * @brief resetPathSlot
     * Reset the path painter
     */
    void resetPathSlot();

private slots:


    /**
     * @brief addPathPointSlot
     * @param name
     * @param x
     * @param y
     * Add a path point to the current path
     */
    void addPathPointSlot(QString name, double x, double y, int waitTime = 0);

    /**
     * @brief deletePathPointSlot
     * @param id
     * Delete a path point from the current path
     */
    void deletePathPointSlot(int ide);

    /**
     * @brief updatePathPainterPointViewSlot
     * Only redraw the pointViews of the path
     */
    void updatePathPainterPointViewSlot();

    /**
     * @brief orderPathPointChangedSlot
     * @param from
     * @param to
     * Re order the path according to the path points that has been dragged & dropped in the pathpoint list
     */
    void orderPathPointChangedSlot(int from, int to);

    /**
     * @brief actionChangedSlot
     * @param id
     * @param action
     * @param waitTime
     * Update the current path when an action changed (wait for human action or wait for some time)
     */
    void actionChangedSlot(int id, QString waitTime);

    /**
     * @brief editPathPointSlot
     * @param id
     * @param name
     * @param x
     * @param y
     * Move the path point with the given name to its new id position
     */
    void editPathPointSlot(int id, QString name, double, double);

signals:
    void updatePoints(int id, QString name);

private:
    QPainterPath path;
    QSharedPointer<Points> points;
    /// changing as the user edits the path of its robot
    QVector<QSharedPointer<PathPoint>> currentPath;
    QVector<QSharedPointer<PathPoint>> oldPath;
    MainWindow* mainWindow;
    bool pathDeleted;
    QString visiblePath;
};

#endif // PATHPAINTER_H
