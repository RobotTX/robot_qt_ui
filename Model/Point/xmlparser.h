#ifndef XMLPARSER_H
#define XMLPARSER_H

class PointController;
class QXmlStreamReader;

#include <QVector>
#include <QSharedPointer>
#include <QFile>


/**
 * @brief The XMLParser class
 * This class provides functions to import robots and points to the model
 * It also provide a function to save the points of our model in an xml file
 * Be careful the functions of this class do not check whether or not you provided the appropriate type of xml files
 */

class XMLParser{

public:
    XMLParser();

public:
    /**
     * @brief save
     * @param points
     * Saves our model in an xml file
     */
    static void save(PointController *pointController, const QString fileName);

    /**
     * @brief readPoints
     * @param points
     * Imports a list of points from an xml file to the model
     */
    static void readPoints(PointController *pointController, const QString fileName);

private:

    /**
     * @brief readNameElement
     * @param xmlReader
     * @return QString
     * Reads a name attribute in an xml file
     */
    static QString readNameElement(QXmlStreamReader &xmlReader);

    /**
     * @brief readCoordinateElement
     * @param xmlReader
     * @return double
     * Reads a coordinate attribute in an xml file
     */
    static double readCoordinateElement(QXmlStreamReader &xmlReader);

    /**
     * @brief readDisplayedElement
     * @param xmlReader
     * @return bool
     * Reads a boolean attribute in an xml file
     */
    static bool readDisplayedElement(QXmlStreamReader &xmlReader);

    /**
     * @brief clear
     * clears the file, only leaving the default group empty
     * careful this function assumes that you are absolutely sure it is applied to a points xml file
     */
    static void clear(PointController* pointController, const QString fileName);
};

#endif /// XMLPARSER_H
