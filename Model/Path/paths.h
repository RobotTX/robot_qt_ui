#ifndef PATHS_H
#define PATHS_H

class PathPoint;

#include <QObject>

class Paths : public QObject {
public:
    Paths(QObject *parent = Q_NULLPTR);

    QMap<QString, QMap<QString, QVector<PathPoint*>*>*>* getGroups(void) const { return groups; }
    void setGroups(QMap<QString, QMap<QString, QVector<PathPoint*>*>*>* _groups) { groups = _groups; }

private:
    QMap<QString, QMap<QString, QVector<PathPoint*>*>*>* groups;
};

/**
 * @brief operator <<
 * @param out
 * @param paths
 * @return
 * Serialization of the paths
 */
QDataStream& operator<<(QDataStream& out, const Paths& paths);

/**
 * @brief operator >>
 * @param in
 * @param paths
 * @return
 * Deserialization of the paths
 */
QDataStream& operator>>(QDataStream& in, Paths& paths);

#endif // PATHS_H
