#include "paths.h"
#include "Controller/mainwindow.h"

Paths::Paths(MainWindow *parent): QObject(parent)
{
    groups = QSharedPointer<Groups>(new Groups());
}

void Paths::displayGroups() const {
    QMapIterator<QString, QSharedPointer<CollectionPaths>> it(*(groups));
    while(it.hasNext()){
        it.next();
        qDebug() << "Group of paths:" << it.key();
        QMapIterator<QString, QSharedPointer<Path> > it_paths(*(it.value()));
        while(it_paths.hasNext()){
            it_paths.next();
            qDebug() << "\tPath:" << it_paths.key();
            qDebug() << "\tPoints";
            if(it_paths.value()){
                for(int i = 0; i < it_paths.value()->size(); i++)
                    qDebug() << "\t" << it_paths.value()->at(i)->getPoint().getName();
            }
        }
    }
}

void Paths::createPath(const QString groupName, const QString pathName){
    auto it_group = groups->find(groupName);
    if(it_group == groups->end())
        qDebug() << "Paths::createPath the group of paths" << groupName << "does not exist";
    else {
        QSharedPointer<QMap<QString, QSharedPointer<Path>> > current_paths = (*groups)[groupName];
        if(current_paths->find(pathName) == current_paths->end())
            current_paths->insert(pathName, QSharedPointer<Path>());
        else
            qDebug() << "The group of paths" << groupName << "already contains a path named" << pathName;
    }
}

void Paths::addPathPoint(const QString groupName, const QString pathName, const QSharedPointer<PathPoint> &pathPoint){
    /// if the group of paths exists
    auto it = groups->find(groupName);
    if(it != groups->end()){
        /// if the path exists
        QSharedPointer<CollectionPaths> paths_ptr = it.value();
        auto it_path = paths_ptr->find(pathName);
        if(it_path != paths_ptr->end()){
            QSharedPointer<Path> current_path_ptr = it_path.value();
            if(current_path_ptr)
                current_path_ptr->push_back(pathPoint);
            else {
                QSharedPointer<Path> new_path_ptr(new Path);
                new_path_ptr->push_back(pathPoint);
                (*(*groups)[groupName])[pathName] = new_path_ptr;
            }

        } else
            qDebug() << "Paths::addPathPoint The path" << pathName << "does not exist";

    } else
        qDebug() << "Paths::addPathPoint the group of paths" << groupName << "does not exist";
}

QDataStream& operator>>(QDataStream& in, Paths& paths){
    qDebug() << "Paths operator>> Deserializing the paths";
    QMap<QString, QMap<QString, QVector<PathPoint>>> tmpPaths;
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

    return in;
}

QDataStream& operator<<(QDataStream& out, const Paths& paths){
    qDebug() << "Paths operator<< Serializing the paths";
    QMap<QString, QMap<QString, QVector<PathPoint>>> tmpPaths;

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
    out << tmpPaths;
    return out;
}
