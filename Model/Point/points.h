#ifndef POINTS_H
#define POINTS_H

class Group;

#include <QObject>
#include <QMap>
#include <QPointer>
#include <QVariant>

#define NO_GROUP_NAME "No Group"

/**
 * @brief The Points class
 * This class provides a model for a list of points organized in groups
 * A Points objet is identified by a vector of pointers in a map of group identified
 * by their group name
 */

class Points : public QObject {
    Q_OBJECT

public:
    Points(QObject *parent);
    QMap<QString, QPointer<Group>> getGroups(void) const { return groups; }

    void addGroup(const QString groupName);
    void addPoint(const QString groupName, const QString name, const double x, const double y, const bool displayed);
    void deletePoint(const QString groupName, const QString name);
    void deleteGroup(const QString groupName);
    void hideShow(const QString groupName, const QString name);
    void renameGroup(const QString newName, const QString oldName);
    void movePoint(const QString name, const QString oldGroup, const QString newGroup);

    /**
     * @brief checkGroupName
     * @param name
     * @return if the given name of group is taken
     */
    bool checkGroupName(const QString name);

private:
    QMap<QString, QPointer<Group>> groups;
};

#endif /// POINTS_H
