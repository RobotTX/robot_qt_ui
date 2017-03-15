#ifndef POINTS_H
#define POINTS_H

class Point;

#include <QObject>
#include <QMap>
#include <QVector>
#include <QVariant>
#include <QPointer>

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
    QMap<QString, QVector<QPointer<Point> >>& getGroups(void) { return groups; }

    void resetGroups(void);
    void addPointToGroup(const QString group, QPointer<Point> point) { groups[group].push_back(point); }
    void deletePointFromGroup(const QString group, const QString point_name);

private:
    QMap<QString, QVector<QPointer<Point> >> groups;
};

#endif /// POINTS_H
