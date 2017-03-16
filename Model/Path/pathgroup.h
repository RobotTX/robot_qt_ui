#ifndef PATHGROUP_H
#define PATHGROUP_H

class Path;

#include <QObject>
#include <QMap>
#include <QPointer>

class PathGroup : public QObject {
public:
    PathGroup(QObject *parent);
    QMap<QString, QPointer<Path>> getPaths(void) const { return paths; }

    void addPath(const QString name);
    void deletePath(const QString name);
    void addPathPoint(const QString pathName, const QString name, const double x, const double y, const int waitTime);
    void deletePathPoint(const QString pathName, const QString name);

private:
    QMap<QString, QPointer<Path>> paths;
};

#endif // PATHGROUP_H
