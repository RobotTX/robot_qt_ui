#include "points.h"
#include "group.h"
#include "point.h"
#include <QDataStream>

Points::Points(void)
{
}

bool Points::addGroup(const Group& group){
    /// we first check that no group with the same name exists
    for(int i = 0; i < groups.size(); i++){
        if(!group.getName().compare(groups.at(i)->getName()))
            /// if there's already a point with the same name we don't do anything and return false
            return false;
    }
    groups.push_back(std::make_shared<Group>(group));
    return true;
}

QVector<QString> Points::groupNames(void) const{
    QVector<QString> _names;
    for(auto it = groups.cbegin(); it != groups.cend(); ++it)
        _names.push_back((*it)->getName());
    return _names;
}

void Points::display(std::ostream& stream) const {
    std::cout << "This list of points contains " << groups.size() << " groups :" << std::endl;
    for(int i = 0; i < groups.size(); i++)
        stream << *(groups[i]) << std::endl;
}

std::ostream& operator <<(std::ostream& stream, Points const& points){
    points.display(stream);
    return stream;
}


QDataStream& operator>>(QDataStream& in, Points& points){
    /// the size of the vector has to be serialized too in order to deserialize the object correctly
    qint32 size;
    in >> size;
    for(int i = 0; i < size; i++){
        Group group;
        in >> group;
        points.addGroup(group);
    }
    return in;
}

QDataStream& operator<<(QDataStream& out, const Points& points){
    out << qint32(points.getGroups().size());
    for(int i = 0; i < points.getGroups().size(); i++)
        out << *(points.getGroups().at(i));
    return out;
}


void Points::removeGroup(const int index) {
    if(index >=0 && index < groups.size())
        groups.remove(index);
}

std::shared_ptr<Group> Points::findGroup(const QString groupName) const {
    for(int i = 0; i < getGroups().size(); i++){
        if(!getGroups().at(i)->getName().compare(groupName))
            return getGroups().at(i);
    }
    return NULL;
}

std::shared_ptr<Point> Points::findPoint(const QString name) const {
    for(int i = 0; i < groups.size(); i++){
        std::shared_ptr<Group> currentGroup = groups.at(i);
        for(int j = 0; j < currentGroup->getPoints().size(); j++){
            std::shared_ptr<Point> currentPoint = currentGroup->getPoints().at(j);
            if(!name.compare(currentPoint->getName()))
                return currentPoint;
        }
    }
    return NULL;
}

std::pair<int, int> Points::findPointIndexes(const QString name) const {
    std::pair<int, int> indexes(std::make_pair<int, int>(-1, -1));
    for(int i = 0; i < groups.size(); i++){
        std::shared_ptr<Group> currentGroup = groups.at(i);
        for(int j = 0; j < currentGroup->getPoints().size(); j++){
            std::shared_ptr<Point> currentPoint = currentGroup->getPoints().at(j);
            if(!name.compare(currentPoint->getName())){
                indexes.first = i;
                indexes.second = j;
            }
        }
    }
    return indexes;
}


void Points::clearGroups(){
    groups.clear();
}
