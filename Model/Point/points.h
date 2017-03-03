#ifndef POINTS_H
#define POINTS_H

class Point;

#include <QObject>
#include <QMap>
#include <QVector>

#define NO_GROUP_NAME "No Group"

/**
 * @brief The Points class
 * This class provides a model for a list of points organized in groups
 * A Points objet is identified by a vector of pointers on Group objets
 */

class Points : public QObject {
    Q_OBJECT

public:
    Points(QObject *parent);

    void addGroup(QString groupName);
    void addPoint(QString groupName, QString name, double x, double y, bool displayed);

private:
    QMap<QString, QVector<Point*>*>* groups;
};

#endif /// POINTS_H
