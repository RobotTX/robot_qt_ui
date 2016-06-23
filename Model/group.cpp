#include "group.h"
#include "points.h"
#include "point.h"
#include <QDebug>
#include <QDataStream>

Group::Group(void){}

Group::Group(const QString _name) :  name(_name) {
}

Group::Group(const std::shared_ptr<Points>& _groupPoints, const QString _name) :  name(_name) {
    groupPoints = _groupPoints;
}

bool Group::addPoint(const std::shared_ptr<Point> pointPtr){
    /// we first check that there is not point with the same name already in the group
    if(!pointPtr->getName().compare(""))
        return -1;

    for(int i = 0; i < points.size(); i++){
        if(!points.at(i)->getName().compare(pointPtr->getName()))
            /// if there's already a point with the same name we don't do anything and return false
            return false;
    }
    points.push_back(pointPtr);
    return true;
}

int Group::addPoint(const QString name, const float x, const float y){
    /// we first check that there is not point with the same name already in the group
    if(!name.compare(""))
        return -1;
    for(int j = 0; j < groupPoints->getGroups().size(); j++){
        std::shared_ptr<Group> currentGroup = groupPoints->getGroups().at(j);
        for(int i = 0; i < currentGroup->getPoints().size(); i++){
            std::shared_ptr<Point> currentPoint = currentGroup->getPoints().at(i);
            if(!currentPoint->getName().compare(name)){
                /// if there's already a point with the same name we don't do anything and return false
                qDebug() << "already here";
                return 0;
            }
        }
    }
    points.push_back(std::make_shared<Point> (Point(name, x, y)));
    return 1;
}

void Group::display(std::ostream& stream) const{
    stream << "Group's name : " << getName().toStdString() << std::endl;
    stream << "This group contains the following points : " << std::endl;
    for(int i = 0; i < points.size(); i++)
        stream << "\t" << *points.at(i) << std::endl;
}

void Group::removePoint(const QString name){
    for(QVector<std::shared_ptr<Point>>::iterator it = points.begin(); it != points.end(); ++it){
        if(!(*it)->getName().compare(name)){
            points.erase(it);
            break;
        }
    }
}

void Group::removePoint(const int index){
    if(index >= 0 && index < points.size())
        points.remove(index);
}

std::ostream& operator <<(std::ostream& stream, const Group& group){
    group.display(stream);
    return stream;
}

QDataStream& operator<<(QDataStream& out, const Group& group){
    out << group.getName();
    out << qint32(group.getPoints().size());
    for(int i = 0; i < group.getPoints().size(); i++)
        out << *(group.getPoints().at(i));
    return out;
}

QDataStream& operator>>(QDataStream& in, Group& group){
    QString name;
    /// the size of the vector has to be serialized too in order to deserialize the object correctly
    qint32 size;
    in >> name >> size;
    std::shared_ptr<Points> groupPoints;
    group = Group(groupPoints, name);
    for(int i = 0; i < size; i++){
        Point point;
        in >> point;
        group.addPoint(std::make_shared<Point> (Point(point)));
    }
    return in;
}

 bool Group::isDisplayed(void) const {
    foreach(std::shared_ptr<Point> point, points)
        if(!point->isDisplayed())
            return false;
    return true;
 }

std::shared_ptr<Point> Group::containsHomePoint(void) const {
     for(int i = 0; i < points.size(); i++){
         if(points[i]->isHome())
             return points[i];
     }
     return 0;
 }
