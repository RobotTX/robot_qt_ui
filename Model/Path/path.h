#ifndef PATH_H
#define PATH_H

class PathPoint;

#include <QObject>
#include <QPointer>
#include <QVector>

class Path : public QObject {
public:
    Path(QObject* parent);

private:
    QVector<QPointer<PathPoint>> pathPointVector;
};

#endif // PATH_H
