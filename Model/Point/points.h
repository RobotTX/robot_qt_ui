#ifndef POINTS_H
#define POINTS_H

class Point;

#include <QObject>
#include <QMap>
#include <QVector>
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
    QMap<QString, QVector<Point*>*>* getGroups(void) const { return groups; }

private:
    QMap<QString, QVector<Point*>*>* groups;
};

#endif /// POINTS_H
