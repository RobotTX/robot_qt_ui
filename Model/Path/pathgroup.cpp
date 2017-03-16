#include "pathgroup.h"
#include "Model/Path/path.h"

PathGroup::PathGroup(QObject* parent) : QObject(parent), paths(QMap<QString, QPointer<Path>>()){

}
