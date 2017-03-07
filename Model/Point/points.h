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
 * A Points objet is identified by a vector of pointers on Group objets
 */

class Points : public QObject {
    Q_OBJECT

public:
    Points(QObject *parent);

    void addGroup(QString groupName);
    void addPoint(QString groupName, QString name, double x, double y, bool displayed);
    int indexOfPoint(QString pointName, QString groupName);
    int indexOfGroup(QString groupName);

private slots:
    void deletePointOrGroup(QString name, QString groupName);

signals:
    void addGroupQml(QVariant index, QVariant name);
    void addPointQml(QVariant index, QVariant name, QVariant isVisible, QVariant groupName, QVariant x, QVariant y);
    void removeGroupQml(QVariant begin, QVariant end);
    void removePointQml(QVariant index);

private:
    QMap<QString, QVector<Point*>*>* groups;
};

#endif /// POINTS_H
