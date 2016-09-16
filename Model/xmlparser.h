#ifndef XMLPARSER_H
#define XMLPARSER_H

class Points;
class Robots;
class QFile;
class QXmlStreamReader;
class MapView;
class MainWindow;

#include <QVector>
#include <QGraphicsItem>
#include <QSharedPointer>


/**
 * @brief The XMLParser class
 * This class provides functions to import robots and points to the model
 * It also provide a function to save the points of our model in an xml file
 * Be careful the functions of this class do not check whether or not you provided the appropriate type of xml files
 */

class XMLParser{

public:
    XMLParser(const QString filename);
    ~XMLParser();

public:
    /**
     * @brief save
     * @param points
     * Saves our model in an xml file
     */
    void save(const Points& points) const;

    /**
     * @brief readPoints
     * @param points
     * Imports a list of points from an xml file to the model
     */
    void readPoints(QSharedPointer<Points>& points);

    /**
     * @brief readNameElement
     * @param xmlReader
     * @return QString
     * Reads a name attribute in an xml file
     */
    QString readNameElement(QXmlStreamReader &xmlReader);

    /**
     * @brief readIPElement
     * @param xmlReader
     * @return QString
     * Reads an IP attribute in an xml file
     */
    QString readIPElement(QXmlStreamReader &xmlReader);

    /**
     * @brief readCoordinateElement
     * @param xmlReader
     * @return float
     * Reads a coordinate attribute in an xml file
     */
    float readCoordinateElement(QXmlStreamReader &xmlReader);

    /**
     * @brief readDisplayedElement
     * @param xmlReader
     * @return bool
     * Reads a boolean attribute in an xml file
     */
    bool readDisplayedElement(QXmlStreamReader &xmlReader);

    /**
     * @brief clear
     * clears the file, only leaving the default group empty
     * careful this function assumes that you are absolutely sure it is applied to a points xml file
     */
    void clear(void);

private:
    /// a pointer to the file that will be opened in order to retrieve or save the model
    QFile* file;
};

#endif // XMLPARSER_H
