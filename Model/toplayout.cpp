#include "toplayout.h"

TopLayout::TopLayout() {}

void TopLayout::removeRobotWithoutHome(const QString name){
    for(int i = 0; i < robotsWithoutHome.size(); i++){
        if(!robotsWithoutHome.at(i).compare(name)){
            robotsWithoutHome.remove(i);
            return;
        }
    }
}

QString TopLayout::getRobotsString() const {
    QString robots_string("");
    for(int i = 0; i < robotsWithoutHome.size(); i++){
        if(!robots_string.isEmpty())
            robots_string += ", ";
        robots_string += robotsWithoutHome.at(i);
    }
    return robots_string;
}

