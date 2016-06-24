#ifndef COMPAREROBOTSXML_H
#define COMPAREROBOTSXML_H


#include <QVector>
#include <QGraphicsItem>

/**
 * @brief The CompareRobotsXml class
 * This class provides a way to determine which robots disconnected and which robots connected since the last update
 * This function is meant to be called regularly in order to keep track of the changes within the robot ecosysten
 */

class CompareRobotsXml
{
public:
    enum State { CONNECTED, DISCONNECTED };

public:
    /**
     * @brief CompareRobotsXml
     * @param _oldFile
     * @param _newFile
     * Compares an old file that contains the last known connected robots and a new file which contains the very last update
     * It returns a vector containing the name of the robots that either went disconnected since the last update and the robots
     * that just connected since the last update along with their state (DISCONNECTED or CONNECTED)
     */
    CompareRobotsXml(const QString _oldFile, const QString _newFile, QGraphicsItem *_parent);

    QVector<std::pair<QString, State> > compare(void) const ;

private:
    QString oldFile;
    QString newFile;
    QGraphicsItem* parent;
};

#endif // COMPAREROBOTSXML_H
