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
    void createGroup(const QString name);
    void createPath(const QString groupName, const QString pathName);
    void addPathPoint(const QString groupName, const QString pathName, const QSharedPointer<PathPoint>& pathPoint);
    void deleteGroup(const QString name);
    void displayGroups(void) const;

private:
    QSharedPointer<Groups> groups;
};

QDataStream& operator<<(QDataStream& out, const Paths& paths);
QDataStream& operator>>(QDataStream& in, Paths& paths);

#endif // PATHS_H
