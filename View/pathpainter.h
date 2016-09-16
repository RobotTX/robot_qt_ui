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
#include "Model/graphicitemstate.h"

/**
 * @brief The PathPainter class
 * The class that draw the lines between each points showing the path of a robot
 */
class PathPainter : public QObject, public QGraphicsPathItem{
    Q_OBJECT

public:
    PathPainter(MainWindow* const &mainWindow, const QSharedPointer<Points> _points, const GraphicItemState _state);

    /// Getters
    QVector<QSharedPointer<PathPoint>> getCurrentPath(void) const { return currentPath; }
    QVector<QSharedPointer<PathPoint>> getOldPath(void) const { return oldPath; }
    bool getPathDeleted(void) const { return pathDeleted; }

    /// Setters
    void setCurrentPath(const QVector<QSharedPointer<PathPoint> > &_currentPath);
    void setOldPath(const QVector<QSharedPointer<PathPoint> > _oldPath);
    void setPathDeleted(const bool _pathDeleted){ pathDeleted = _pathDeleted; }

    /**
     * @brief displayPath
     * Display the current path
     */
    void displayPath(void);

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

private slots:
    /**
     * @brief resetPathSlot
     * @param _state
     * Reset the path painter
     */
    void resetPathSlot(GraphicItemState _state);

    /**
     * @brief addPathPointSlot
     * @param name
     * @param x
     * @param y
     * Add a path point to the current path
     */
    void addPathPointSlot(QString name, double x, double y, int action = 0, int waitTime = 0);

    /**
     * @brief deletePathPointSlot
     * @param id
     * @param _state
     * Delete a path point from the current path
     */
    void deletePathPointSlot(int id, GraphicItemState _state);

    /**
     * @brief updatePathPainterSlot
     * @param _state
     * @param savePath
     * Redraw the whole path
     */
    void updatePathPainterSlot(GraphicItemState _state, const bool savePath);

    /**
     * @brief updatePathPainterPointViewSlot
     * @param _state
     * Only redraw the pointViews of the path
     */
    void updatePathPainterPointViewSlot(GraphicItemState _state);

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
    void actionChangedSlot(int id, int action, QString waitTime);

    /**
     * @brief editPathPointSlot
     * @param id
     * @param name
     * @param x
     * @param y
     * Move the path point with the given name to its new id position
     */
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
    bool pathDeleted;
    /// to hold whether one instance is drawing paths related to robots or not
    const GraphicItemState state;
};

#endif // PATHPAINTER_H
