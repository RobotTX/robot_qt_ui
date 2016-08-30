#include "paths.h"
#include "Controller/mainwindow.h"

Paths::Paths(MainWindow *parent): QObject(parent), visiblePath("")
{
    groups = QSharedPointer<Groups>(new Groups());
}

/// Simple function to display the content of a path object
void Paths::displayGroups() const {
    qDebug() << "\nPaths::displayGroups called";
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
    qDebug() << "";
}

/// to create a path in the group of paths indexed by <groupName>
/// if the object does not contain a group of paths named <groupName> or if a path called <pathName> already exists
/// this function does not do anything, otherwise it creates an empty path which name is <pathName>
/// in the group of paths which name is <groupName>
void Paths::createPath(const QString groupName, const QString pathName){
    auto it_group = groups->find(groupName);
    if(it_group == groups->end())
        qDebug() << "Paths::createPath the group of paths" << groupName << "does not exist";
    else {
        QSharedPointer<QMap<QString, QSharedPointer<Path>> > current_paths = (*groups)[groupName];
        if(current_paths->find(pathName) == current_paths->end())
            current_paths->insert(pathName, QSharedPointer<Path>(new Path()));
        else
            qDebug() << "The group of paths" << groupName << "already contains a path named" << pathName;
    }
}

/// to add a path point to the path called <pathName> in the group of paths <groupName>
/// if either one of the group of paths or path itself does not exist, this function does not do anything
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

/// attempts to create a group of paths called <name>, if the group already exists the function does not do anything
void Paths::createGroup(const QString name){
    if(groups->find(name) == groups->end())
        groups->insert(name,
                       QSharedPointer<CollectionPaths>(new CollectionPaths));
    else
        qDebug() << "A group named" << name << "already exists";
}

/// attempts to delete a group of paths called <name>, if the group does not exist, the function does not do anything
void Paths::deleteGroup(const QString name){
    qDebug() << "Paths::deleteGroup called";
    if(groups->find(name) != groups->end()){
        int r = groups->remove(name);
        qDebug() << "removed" << r << "value(s) with key" << name;
    } else
        qDebug() << name << "is not in the map";
}

/// attempts to delete a path with name <pathName> in the group <groupName>, does not
/// do anything if such path does not exist
void Paths::deletePath(const QString groupName, const QString pathName){

    qDebug() << "Paths::deletePath called";
    auto it_group = groups->find(groupName);
    if(it_group == groups->end())
        qDebug() << "Paths::deletePath the group of paths" << groupName << "does not exist";
    else {
        QSharedPointer<QMap<QString, QSharedPointer<Path>> > current_paths = (*groups)[groupName];
        int rem = current_paths->remove(pathName);
        if(rem == 0)
            qDebug() << "Paths::deletePath the path" << pathName << "does not exist within the group" << groupName;
    }
}

Paths::Path Paths::getPath(const QString groupName, const QString pathName, bool& foundFlag){
    qDebug() << "Paths::getPath called";
    auto it_group = groups->find(groupName);
    if(it_group == groups->end())
        qDebug() << "Paths::getPath the group of paths" << groupName << "does not exist";
    else {
        QSharedPointer<QMap<QString, QSharedPointer<Path>> > current_paths = (*groups)[groupName];
        if(current_paths->find(pathName) != current_paths->end()){
            foundFlag = true;
            return *(current_paths->find(pathName).value());
        }
        else
            qDebug() << "Paths::getPath the path" << pathName << "does not exist within the group" << groupName;
    }
    return Path();
}

QDataStream& operator>>(QDataStream& in, Paths& paths){
    //qDebug() << "\nPaths operator>> Deserializing the paths";

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
            for(int i = 0; i < it_paths.value().size(); i++){
                paths.addPathPoint(it.key(), it_paths.key(), QSharedPointer<PathPoint>(new PathPoint(it_paths.value().at(i))));
            }
        }
    }

    return in;
}

QDataStream& operator<<(QDataStream& out, const Paths& paths){
    //qDebug() << "\nPaths operator<< Serializing the paths";

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
                for(int i = 0; i < it_paths.value()->size(); i++){
                    tmpPath.push_back(*(it_paths.value()->at(i)));
                }
            }
            tmpGroup.insert(it_paths.key(), tmpPath);
        }
        tmpPaths.insert(it.key(), tmpGroup);
    }
    out << tmpPaths;
    return out;
}
