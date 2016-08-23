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
    typedef QVector<QSharedPointer<PathPoint> > Path;
    typedef QMap<QString, QSharedPointer<Path> > CollectionPaths;
    typedef QMap<QString, QSharedPointer<CollectionPaths> > Groups;

public:
    Paths(MainWindow *parent);

    QSharedPointer<Groups> getGroups(void) const { return groups; }
    void createGroup(const QString name);
    void createPath(const QString groupName, const QString pathName);
    void addPathPoint(const QString groupName, const QString pathName, const QSharedPointer<PathPoint>& pathPoint);

    void displayGroups(void) const;

private:
    QSharedPointer<Groups> groups;
};

inline void Paths::createGroup(const QString name){
    if(groups->find(name) == groups->end())
        groups->insert(name,
                       QSharedPointer<CollectionPaths>(new CollectionPaths));
    else
        qDebug() << "A group named" << name << "already exists";
}

#endif // PATHS_H
