#ifndef PATHS_H
#define PATHS_H

class MainWindow;

#include <QMap>
#include <QVector>
#include <QSharedPointer>
#include <QString>
#include <QObject>
#include <QDebug>

#include "Model/pathpoint.h"

/**
 * @brief The Paths class
 * Contains all the paths saved by the user (those which may or may not assigned to a robot or several robots)
 * The structure of a Paths object :
 * Map of Groups of paths indexed by their names
 *      Map of paths indexed by their names as well
 *          paths : vector of path points
 */
class Paths: public QObject {
    Q_OBJECT

public:
    typedef QVector<QSharedPointer<PathPoint>> Path;
    typedef QMap<QString, QSharedPointer<Path>> CollectionPaths;
    typedef QMap<QString, QSharedPointer<CollectionPaths>> Groups;

public:
    Paths(MainWindow *parent = Q_NULLPTR);

    /**
     * @brief createGroup
     * @param groupName
     * Create the group of paths groupName
     */
    void createGroup(const QString groupName);

    /**
     * @brief createPath
     * @param groupName
     * @param pathName
     * @return
     * Create the path pathName in the group groupName
     */
    bool createPath(const QString groupName, const QString pathName);

    /**
     * @brief addPathPoint
     * @param groupName
     * @param pathName
     * @param pathPoint
     * Add pathPoint to the path pathName in the group groupName
     */
    void addPathPoint(const QString groupName, const QString pathName, const QSharedPointer<PathPoint>& pathPoint);

    /**
     * @brief deleteGroup
     * @param groupName
     * Delete the group of paths groupName
     */
    void deleteGroup(const QString groupName);

    /**
     * @brief displayGroups
     * Display all the groups
     */
    void displayGroups(void) const;

    /**
     * @brief deletePath
     * @param groupName
     * @param pathName
     * Delete pathName in the given group
     */
    void deletePath(const QString groupName, const QString pathName);

    /**
     * @brief containsPoint
     * @param groupName
     * @param pathName
     * @return bool
     * returns true if the path <pathName> of the group <groupName> contains the point <point>
     */
    bool containsPoint(const QString groupName, const QString pathName, const Point& point);


    /// Getters
    QSharedPointer<Groups> getGroups(void) const { return groups; }
    /// the foundFlag is set within the function, after the function returns it holds true if the path has been found and false otherwise
    Path getPath(const QString groupName, const QString pathName, bool& foundFlag);

    /// Setters
    void setGroups(QSharedPointer<Groups> _groups) { groups = _groups; }

    /// updates the paths which contains the pathpoint whose name is pointName to change the name to the points coordinates
    /// instead. Has to be done when the original point has been modified (edition of a point)
    void updatePaths(const Point& old_point, const Point& new_point);


private:
    QSharedPointer<Groups> groups;
};


/**
 * @brief operator <<
 * @param out
 * @param paths
 * @return
 * Serialization of the paths
 */
QDataStream& operator<<(QDataStream& out, const Paths& paths);

/**
 * @brief operator >>
 * @param in
 * @param paths
 * @return
 * Deserialization of the paths
 */
QDataStream& operator>>(QDataStream& in, Paths& paths);

#endif // PATHS_H
