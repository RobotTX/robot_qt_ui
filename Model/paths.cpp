#include "paths.h"
#include "Controller/mainwindow.h"

Paths::Paths(MainWindow *parent): QObject(parent)
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
bool Paths::createPath(const QString groupName, const QString pathName){
    auto it_group = groups->find(groupName);
    if(it_group == groups->end())
        qDebug() << "Paths::createPath the group of paths" << groupName << "does not exist";
    else {
        QSharedPointer<QMap<QString, QSharedPointer<Path>> > current_paths = (*groups)[groupName];
        if(current_paths->find(pathName) == current_paths->end()){
            current_paths->insert(pathName, QSharedPointer<Path>(new Path()));
            return true;
        }
        else
            return false;
    }
    return false;
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
void Paths::deleteGroup(const QString groupName){
    qDebug() << "Paths::deleteGroup called";
    if(groups->find(groupName) != groups->end()){
        int r = groups->remove(groupName);
        qDebug() << "removed" << r << "value(s) with key" << groupName;
    } else
        qDebug() << groupName << "is not in the map";
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
    //qDebug() << "Paths::getPath called";
    auto it_group = groups->find(groupName);
    if(it_group == groups->end()){
        qDebug() << "Paths::getPath the group of paths" << groupName << "does not exist";
    } else {
        QSharedPointer<QMap<QString, QSharedPointer<Path>> > current_paths = (*groups)[groupName];
        if(current_paths->find(pathName) != current_paths->end()){
            foundFlag = true;
            return *(current_paths->find(pathName).value());
        } else {
            qDebug() << "Paths::getPath the path" << pathName << "does not exist within the group" << groupName;
        }
    }
    return Path();
}

Paths::CollectionPaths Paths::getGroup(const QString groupName){
    auto it_group = groups->find(groupName);
    if(it_group == groups->end()){
        qDebug() << "Paths::getPath the group of paths" << groupName << "does not exist";
        return CollectionPaths();
    }
    else
        return *it_group.value();
}

QDataStream& operator>>(QDataStream& in, Paths& paths){
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
    QMap<QString, QMap<QString, QVector<PathPoint>>> tmpPaths;

    QMapIterator<QString, QSharedPointer<Paths::CollectionPaths>> it(*(paths.getGroups()));
    while(it.hasNext()){
        it.next();
        QMap<QString, QVector<PathPoint>> tmpGroup;
        QMapIterator<QString, QSharedPointer<Paths::Path> > it_paths(*(it.value()));
        qDebug() << "new group" << it.key();
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

bool Paths::containsPoint(const QString groupName, const QString pathName, const Point &point){
    bool foundFlag(false);
    Path path = getPath(groupName, pathName, foundFlag);
    for(int i = 0; i < path.size(); i++){
        if(path.at(i)->getPoint() == point)
            return true;
    }
    return false;
}

void Paths::updatePaths(const Point& old_point, const Point& new_point){
    QMapIterator<QString, QSharedPointer<CollectionPaths>> it(*(groups));
    qDebug() << "new point" << new_point.getName();
    while(it.hasNext()){
        it.next();
        qDebug() << "Group of paths:" << it.key();
        QMapIterator<QString, QSharedPointer<Path> > it_paths(*(it.value()));
        while(it_paths.hasNext()){
            it_paths.next();
            if(it_paths.value()){
                for(int i = 0; i < it_paths.value()->size(); i++){
                    if(it_paths.value()->at(i)->getPoint().comparePos(old_point.getPosition())){

                        qDebug() << "looking at the point " << it_paths.value()->at(i)->getPoint().getName();
                        /// if the point has been deleted we delete it from the path
                        if(!new_point.getName().compare("")){
                            qDebug() << "gotta delete";
                            Point newPoint(QString::number(it_paths.value()->at(i)->getPoint().getPosition().getX()) + "; " +
                                           QString::number(it_paths.value()->at(i)->getPoint().getPosition().getY()),
                                           it_paths.value()->at(i)->getPoint().getPosition());
                            it_paths.value()->at(i)->setPoint(newPoint);
                        }
                        /// if the point has been modified we need to update it
                        else {
                            /// this point is the old one
                            if(!it_paths.value()->at(i)->getPoint().getName().compare(new_point.getName())){
                                qDebug() << "only the position has changed";
                                /// if the position has been modified but not the name
                                Point newPoint(QString::number(it_paths.value()->at(i)->getPoint().getPosition().getX()) + "; " +
                                               QString::number(it_paths.value()->at(i)->getPoint().getPosition().getY()),
                                               it_paths.value()->at(i)->getPoint().getPosition());
                                it_paths.value()->at(i)->setPoint(newPoint);
                            }
                            /// if the name has been modified but not the position
                            else if(it_paths.value()->at(i)->getPoint().getName().compare(new_point.getName()) && it_paths.value()->at(i)->getPoint().comparePos(new_point.getPosition())){
                                qDebug() <<  "only the name has changed";
                                Point newPoint(new_point);
                                it_paths.value()->at(i)->setPoint(newPoint);
                            }
                            /// if both the name and the position has been modified we do the same thing as if only the position had been modified
                            else {
                                Point newPoint(QString::number(it_paths.value()->at(i)->getPoint().getPosition().getX()) + "; " +
                                               QString::number(it_paths.value()->at(i)->getPoint().getPosition().getY()),
                                               it_paths.value()->at(i)->getPoint().getPosition());
                                it_paths.value()->at(i)->setPoint(newPoint);
                                qDebug() << "Everything has changed" << old_point.getName();
                            }
                        }
                    }
                }
            }
        }
    }
}

void Paths::clear(){
    QMapIterator<QString, QSharedPointer<CollectionPaths>> it(*(groups));
    while(it.hasNext()){
        it.next();
        it.value()->clear();
    }
    groups->clear();
}

QPair<QString, QString> Paths::findPath(const QVector<PathPoint>& path) const {
    QMapIterator<QString, QSharedPointer<Paths::CollectionPaths>> it(*(groups));
    while(it.hasNext()){
        it.next();
        QMapIterator<QString, QSharedPointer<Paths::Path> > it_paths(*(it.value()));
        qDebug() << "new group" << it.key();
        while(it_paths.hasNext()){
            it_paths.next();
            qDebug() << "new path" << it_paths.key();
            bool right_path(true);
            if(it_paths.value()){
                /// if the paths don't have the same number of points no need to check
                if(it_paths.value()->size() != path.size())
                    continue;
                for(int i = 0; i < it_paths.value()->size(); i++){
                    /// if one pathPoint is not the same we jump to the next path
                    if(*(it_paths.value()->at(i)) != path.at(i)){
                        std::cout << "these two points are different" << *(it_paths.value()->at(i)) <<
                                    path.at(i);
                        right_path = false;
                    }
                }
                /// but if all points match then we return the name of the group and the name of the path
                if(right_path)
                    return QPair<QString, QString> (it.key(), it_paths.key());
                else
                    break;
            }
        }
    }
    /// if after trying all paths we haven't found any that matched the path given we return ("", "")
    return QPair<QString, QString> ("", "");
}

