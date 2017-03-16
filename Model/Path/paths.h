#ifndef PATHS_H
#define PATHS_H

class PathGroup;

#include <QObject>
#include <QMap>
#include <QPointer>

class Paths : public QObject {
public:
    Paths(QObject *parent);

    QMap<QString, QPointer<PathGroup>> getGroups(void) const { return groups; }

private:
    QMap<QString, QPointer<PathGroup>> groups;
};

#endif // PATHS_H
