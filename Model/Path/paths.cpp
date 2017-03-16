#include "paths.h"
#include <QMap>
#include "Model/Path/pathpoint.h"

Paths::Paths(QObject* parent) : QObject(parent), groups(new QMap<QString, QMap<QString, QVector<PathPoint*>*>*>()) {

}
