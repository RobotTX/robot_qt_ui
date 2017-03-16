#include "path.h"
#include "Model/Path/pathpoint.h"

Path::Path(QObject *parent) : QObject(parent), pathPointVector(QVector<QPointer<PathPoint>>()){

}
