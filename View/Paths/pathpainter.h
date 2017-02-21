#ifndef PATHPAINTER_H
#define PATHPAINTER_H

class MapView;
class Points;
class MainWindow;
class RobotView;
class PointView;

#include <QObject>
#include <QGraphicsPathItem>
#include <QPainterPath>
#include <QPen>
#include <QVector>
#include <QSharedPointer>
#include "Model/Points/point.h"
#include "Model/Paths/pathpoint.h"
#include "Model/Other/graphicitemstate.h"

/**
 * @brief The PathPainter class
 * The class that draw the lines between each points showing the path of a robot
 */
class PathPainter : public QObject, public QGraphicsPathItem{
    Q_OBJECT

public:
    PathPainter(MainWindow* const &mainWindow);

    QVector<QSharedPointer<PathPoint>> getCurrentPath(void) const { return currentPath; }
    QVector<QSharedPointer<PathPoint>> getOldPath(void) const { return oldPath; }
    bool getPathDeleted(void) const { return pathDeleted; }
    QString getVisiblePath(void) const { return visiblePath; }
    QString getRobotName(void) const { return robotName; }

    void setCurrentPath(const QSharedPointer<Points> points, const QPointer<RobotView> robotView, const GraphicItemState state, const QVector<QSharedPointer<PathPoint> > &_currentPath, QString pathName);
    void setOldPath(const QVector<QSharedPointer<PathPoint> > _oldPath);
    void setPathDeleted(const bool _pathDeleted){ pathDeleted = _pathDeleted; }
    void setVisiblePath(const QString path) { visiblePath = path; }
    void setRobotName(const QString _robotName) { robotName = _robotName; }

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
    void updateCurrentPath(QSharedPointer<Points> points);

    /**
     * @brief clearOldPath
     * Delete the content of the old path
     */
    void clearOldPath();

    void resetAllPixmap();

    /**
     * @brief addPathPoint
     * Add a path point to the current path
     */
    void addPathPoint(QSharedPointer<Points> points, QPointer<RobotView> robotView, GraphicItemState state, QString name, double x, double y, int waitTime = 0);

    /**
     * @brief updatePathPainter
     * @param savePath
     * Redraw the whole path
     */
    void updatePathPainter(const QSharedPointer<Points> points, const bool savePath);

    /**
     * @brief editPathPoint
     * @param id
     * @param name
     * @param x
     * @param y
     * Move the path point with the given name to its new id position
     */
    void editPathPoint(const QSharedPointer<Points> points, int id, QString name);

    /**
     * @brief deletePathPoint
     * @param id
     * Delete a path point from the current path
     */
    void deletePathPoint(QSharedPointer<Points> points, int id);

    /**
     * @brief orderPathPointChanged
     * @param from
     * @param to
     * Re order the path according to the path points that has been dragged & dropped in the pathpoint list
     */
    void orderPathPointChanged(QSharedPointer<Points> points, int from, int to);

private slots:


    /**
     * @brief resetPathSlot
     * Reset the path painter
     */
    void resetPathSlot(QSharedPointer<Points> points);

    /**
     * @brief updatePathPainterPointViewSlot
     * Only redraw the pointViews of the path
     */
    void updatePathPainterPointViewSlot(const QSharedPointer<QVector<QSharedPointer<PointView> > > group);

    /**
     * @brief actionChangedSlot
     * @param id
     * @param action
     * @param waitTime
     * Update the current path when an action changed (wait for human action or wait for some time)
     */
    void actionChangedSlot(int id, QString waitTime);

protected:
    /**
     * @brief updatePathPainterName
     * Update the name of the points in the current path
     */
    void updatePathPainterName(QSharedPointer<Points> points);

signals:
    void updatePoints(int id, QString name);

private:
    QPainterPath path;
    /// changing as the user edits the path of its robot
    QVector<QSharedPointer<PathPoint>> currentPath;
    QVector<QSharedPointer<PathPoint>> oldPath;
    bool pathDeleted;
    QString visiblePath;
    QString robotName;
};

#endif // PATHPAINTER_H
