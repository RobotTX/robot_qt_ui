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
#include "View/pointview.h"

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

    QSharedPointer<Groups> getGroups(void) const { return groups; }

    void setGroups(QSharedPointer<Groups> _groups) { groups = _groups; }

    /**
     * @brief createGroup
     * @param groupName
     * Create the group of paths groupName
     */
    bool createGroup(const QString groupName);

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
    int deleteGroup(const QString groupName);

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
    int deletePath(const QString groupName, const QString pathName);

    /**
     * @brief containsPoint
     * @param groupName
     * @param pathName
     * @return bool
     * returns true if the path <pathName> of the group <groupName> contains the point <point>
     */
    bool containsPoint(const QString groupName, const QString pathName, const Point& point);

    /**
     * @brief getPath
     * @param groupName
     * @param pathName
     * @param foundFlag
     * @return Path aka QVector<QSharedPointer<PathPoint>>
     * returns the path identified by <groupName> and <pathName>
     * foundFlag is set to true if the path has been found and false
     * if it does not exist
     */
    Path getPath(const QString groupName, const QString pathName, bool& foundFlag);

    /**
     * @brief getGroup
     * @param groupName
     * @return CollectionPaths aka QMap<QString, QSharedPointer<Path>>
     * returns the group of paths identified by the name <groupName>
     * returns an empty group if it does not exist
     */
    CollectionPaths getGroup(const QString groupName);

    /**
     * @brief updatePaths
     * @param old_point
     * @param new_point
     * Updates all paths which contain <old_point>
     */
    void updatePaths(const Point& old_point, const Point& new_point);

    /**
     * @brief clear
     * clears all paths and groups
     */
    void clear(void);

    /**
     * @brief findPath
     * @param path
     * @return QPair<QString, QString>
     * returns a pair <groupName, pathName> corresponding to the path
     * given as a parameter
     */
    QPair<QString, QString> findPath(const QVector<PathPoint>& path) const;

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


#endif /// PATHS_H
