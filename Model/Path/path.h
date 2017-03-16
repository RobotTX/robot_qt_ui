#ifndef PATH_H
#define PATH_H

class PathPoint;

#include <QObject>
#include <QPointer>
#include <QVector>

class Path : public QObject {
public:
    Path(QObject* parent);
    QVector<QPointer<PathPoint>> getPathPointVector(void) const { return pathPointVector; }

    void addPathPoint(const QString name, const double x, const double y, const int waitTime);
    void deletePathPoint(const QString name);

private:
    QVector<QPointer<PathPoint>> pathPointVector;
};

#endif // PATH_H
