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

    QSharedPointer<Groups> getGroups(void) const { return groups; }
    QString getVisiblePath(void) const { return visiblePath; }
    void setVisiblePath(const QString path) { visiblePath = path; qDebug() << path << "path displayed"; }
    void setGroups(QSharedPointer<Groups> _groups) { groups = _groups; }
    void createGroup(const QString name);
    void createPath(const QString groupName, const QString pathName);
    void addPathPoint(const QString groupName, const QString pathName, const QSharedPointer<PathPoint>& pathPoint);
    void deleteGroup(const QString name);
    void displayGroups(void) const;
    void deletePath(const QString groupName, const QString pathName);
    /// the foundFlag is set within the function, after the function returns it holds true if the path has been found and false otherwise
    Path getPath(const QString groupName, const QString pathName, bool& foundFlag);


private:
    QString visiblePath;
    QSharedPointer<Groups> groups;
};

/// to serialize, deserialize Path objects
QDataStream& operator<<(QDataStream& out, const Paths& paths);
QDataStream& operator>>(QDataStream& in, Paths& paths);

#endif // PATHS_H
