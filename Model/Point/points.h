#ifndef POINTS_H
#define POINTS_H

class PointGroup;

#include <QObject>
#include <QMap>
#include <QPointer>
#include <QVariant>

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

    QMap<QString, QPointer<PointGroup>> getGroups(void) const { return groups; }

    void addGroup(const QString groupName);
    void deleteGroup(const QString groupName);
    void addPoint(const QString groupName, const QString name, const double x, const double y, const bool displayed);
    void deletePoint(const QString groupName, const QString name);
    void hideShow(const QString groupName, const QString name);
    void renameGroup(const QString newName, const QString oldName);
    void movePoint(const QString name, const QString oldGroup, const QString newGroup);

    /**
     * @brief checkGroupName
     * @param name
     * @return if the given name of group is taken
     */
    bool checkGroupName(const QString name);
    void clearGoups(void);

private:
    QMap<QString, QPointer<PointGroup>> groups;
};

#endif /// POINTS_H
