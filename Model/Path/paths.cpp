#include "paths.h"
#include <QMap>
#include "Model/Path/pathpoint.h"

Paths::Paths(QObject* parent) : QObject(parent), groups(new QMap<QString, QMap<QString, QVector<PathPoint*>*>*>())
{

}

/// TODO paths here
/// deserialization
QDataStream& operator>>(QDataStream& in, Paths& paths){
    /*QMap<QString, QMap<QString, QVector<PathPoint>>> tmpPaths;
    in >> tmpPaths;

    QMapIterator<QString, QMap<QString, QVector<PathPoint>>> it(tmpPaths);
    while(it.hasNext()){
        it.next();
        paths.createGroup(it.key());
        QMapIterator<QString, QVector<PathPoint>> it_paths(it.value());
        while(it_paths.hasNext()){
            it_paths.next();
            paths.createPath(it.key(), it_paths.key());
            for(int i = 0; i < it_paths.value().size(); i++)
                paths.addPathPoint(it.key(), it_paths.key(), QSharedPointer<PathPoint>(new PathPoint(it_paths.value().at(i))));
        }
    }
*/
    return in;
}

/// serialization
QDataStream& operator<<(QDataStream& out, const Paths& paths){
    /*QMap<QString, QMap<QString, QVector<PathPoint>>> tmpPaths;

    QMapIterator<QString, QSharedPointer<Paths::CollectionPaths>> it(*(paths.getGroups()));
    while(it.hasNext()){
        it.next();
        QMap<QString, QVector<PathPoint>> tmpGroup;
        QMapIterator<QString, QSharedPointer<Paths::Path> > it_paths(*(it.value()));
        while(it_paths.hasNext()){
            it_paths.next();
            QVector<PathPoint> tmpPath;
            if(it_paths.value()){
                for(int i = 0; i < it_paths.value()->size(); i++)
                    tmpPath.push_back(*(it_paths.value()->at(i)));
            }
            tmpGroup.insert(it_paths.key(), tmpPath);
        }
        tmpPaths.insert(it.key(), tmpGroup);
    }
    out << tmpPaths;*/
    return out;
}
