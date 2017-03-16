#include "paths.h"
#include <QMap>
#include "Model/Path/pathgroup.h"

Paths::Paths(QObject* parent) : QObject(parent), groups(QMap<QString, QPointer<PathGroup>>()) {

}
