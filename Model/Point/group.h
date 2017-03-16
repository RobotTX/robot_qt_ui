#ifndef GROUP_H
#define GROUP_H

class Point;

#include <QObject>
#include <QPointer>
#include <QVector>

class Group : public QObject {
public:
    Group(QObject *parent);
    QVector<QPointer<Point>> getPointVector(void) const { return pointVector; }

    void addPoint(const QString name, const double x, const double y, const bool displayed);
    void addPoint(const QPointer<Point> point);
    void deletePoint(const QString name);
    void hideShow(const QString name);
    QPointer<Point> takePoint(const QString name);

private:
    QVector<QPointer<Point>> pointVector;
};

#endif // GROUP_H
