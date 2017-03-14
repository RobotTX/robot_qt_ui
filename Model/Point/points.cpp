#include "points.h"
#include <QDataStream>
#include <QDebug>
#include <QString>
#include "Model/Point/point.h"

Points::Points(QObject* parent) : QObject(parent), groups(new QMap<QString, QVector<Point*>*>()) {

}
