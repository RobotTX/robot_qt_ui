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
        }
    }
}

void Paths::createPath(const QString groupName, const QString pathName){
    if(groups->find(groupName) == groups->end())
        qDebug() << "the group of paths" << groupName << "does not exist";
    else
        (*groups)[groupName]->insert(pathName, QSharedPointer<Path>());
}

void Paths::addPathPoint(const QString groupName, const QString pathName, const QSharedPointer<PathPoint> &pathPoint){
    /// if the group of paths exists
    auto it = groups->find(groupName);
    if(it != groups->end()){
        /// if the path exists
        QSharedPointer<CollectionPaths> paths_ptr = it.value();
        auto it_path = paths_ptr->find(pathName);
        if(it_path != paths_ptr->end()){
            qDebug() << it_path.key() << it_path.value();
            QSharedPointer<Path> current_path_ptr = it_path.value();
            if(current_path_ptr)
                current_path_ptr->push_back(pathPoint);
            else {
                QSharedPointer<Path> new_path_ptr(new Path);
                new_path_ptr->push_back(pathPoint);
                (*(*groups)[groupName])[pathName] = new_path_ptr;
            }

        } else
            qDebug() << "The path" << pathName << "does not exist";

    } else
        qDebug() << "the group of paths" << groupName << "does not exist";
}
