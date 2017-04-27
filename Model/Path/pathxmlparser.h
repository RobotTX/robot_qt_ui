#ifndef PATHXMLPARSER_H
#define PATHXMLPARSER_H

class QXmlStreamReader;
class PathController;

#include <QString>

class PathXMLParser {
public:
    PathXMLParser();

public:
    /**
     * @brief save
     * @param paths
     * Saves our model in an xml file
     */
    static void save(PathController *pathController, const QString fileName);

    /**
     * @brief readPaths
     * @param paths
     * Imports a list of paths from an xml file to the model
     */
    static void readPaths(PathController *pathController, const QString fileName);

private:

    static QString readNameElement(QXmlStreamReader &xmlReader);
    static float readCoordinateElement(QXmlStreamReader &xmlReader);
    static int readWaitTimeElement(QXmlStreamReader &xmlReader);
    static void clear(PathController *pathController, const QString fileName);
};

#endif /// PATHXMLPARSER_H
