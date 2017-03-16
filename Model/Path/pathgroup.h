#ifndef PATHGROUP_H
#define PATHGROUP_H

class Path;

#include <QObject>
#include <QMap>
#include <QPointer>

class PathGroup : public QObject {
public:
    PathGroup(QObject *parent);

private:
    QMap<QString, QPointer<Path>> paths;
};

#endif // PATHGROUP_H
