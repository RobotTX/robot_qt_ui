#include "comparerobotsxml.h"
#include "robots.h"
#include "xmlparser.h"

CompareRobotsXml::CompareRobotsXml(const QString _oldFile, const QString _newFile): oldFile(_oldFile), newFile(_newFile) {}

QVector<std::pair<QString, CompareRobotsXml::State>> CompareRobotsXml::compare(void) const {

    /// the vector that contains the differences between the two files which describe the list of robots which
    /// where connected at two different times
    QVector<std::pair<QString, State>> differences;
    XMLParser oldFileParser(oldFile);
    XMLParser newFileParser(newFile);

    Robots oldRobots, newRobots;
    /// we start by storing the names of our robots, as names are unique this is enough to identify them
    QVector<QString> oldRobotsNames = oldFileParser.readRobots(oldRobots);
    QVector<QString> newRobotsNames = newFileParser.readRobots(newRobots);

    for(int i = 0; i < oldRobotsNames.size(); i++){
        State state (DISCONNECTED);
        for(int j = 0; j < newRobotsNames.size(); j++){
            /// We found the old robot among the new ones
            if(!oldRobotsNames.at(i).compare(newRobotsNames.at(j)))
                state = CONNECTED;
        }
        /// We did not find it so we add it to the list of the robots which are now disconnected
        if(state == DISCONNECTED) differences.push_back(std::make_pair(oldRobotsNames.at(i), DISCONNECTED));
    }

    for(int i = 0; i < newRobotsNames.size(); i++){
        State state(CONNECTED);
        for(int j = 0; j < oldRobotsNames.size(); j++){
            /// We found the new robot among the old ones
            if(!newRobotsNames.at(i).compare(oldRobotsNames.at(j)))
                state = DISCONNECTED;
        }
        /// We did not find it so we add it to the list of the robots which recently connected
        if(state == CONNECTED) differences.push_back(std::make_pair(newRobotsNames.at(i), CONNECTED));
    }
    return differences;
}
