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

    void addGroup(const QString groupName);
    void deleteGroup(const QString groupName);
    void addPath(const QString groupName, const QString name);
    void deletePath(const QString groupName, const QString name);
    void addPathPoint(const QString groupName, const QString pathName, const QString name, const double x, const double y, const int waitTime);
    void deletePathPoint(const QString groupName, const QString pathName, const QString name);
    void renameGroup(const QString newName, const QString oldName);
    void movePath(const QString name, const QString oldGroup, const QString newGroup);

    /**
     * @brief checkGroupName
     * @param name
     * @return if the given name of group is taken
     */
    bool checkGroupName(const QString name);
    void clearGroups(void);
    void display(void);

private:
    QMap<QString, QPointer<PathGroup>> groups;
};

#endif /// PATHS_H
