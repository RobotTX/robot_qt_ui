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

void Paths::createGroup(const QString name){
    if(groups->find(name) == groups->end())
        groups->insert(name,
                       QSharedPointer<CollectionPaths>(new CollectionPaths));
    else
        qDebug() << "A group named" << name << "already exists";
}

void Paths::deleteGroup(const QString name){
    qDebug() << "Paths::deleteGroup called";
    if(groups->find(name) != groups->end()){
        int r = groups->remove(name);
        qDebug() << "removed" << r << "value(s) with key" << name;
    } else
        qDebug() << name << "is not in the map";
}
