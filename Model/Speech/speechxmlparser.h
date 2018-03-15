#ifndef SPEECHXMLPARSER_H
#define SPEECHXMLPARSER_H

class SpeechController;
class QXmlStreamReader;

#include <QVector>
#include <QSharedPointer>
#include <QFile>

/**
 * @brief The SpeechXMLParser class
 * This class provides functions to import robots and speechs to the model
 * It also provide a function to save the speechs of our model in an xml file
 * Be careful the functions of this class do not check whether or not you provided the appropriate type of xml files
 */

class SpeechXMLParser {

public:
    SpeechXMLParser();

public:
    /**
     * @brief save
     * @param speechs
     * Saves our model in an xml file
     */
    static void save(SpeechController *speechController, const QString fileName);

    /**
     * @brief readSpeechs
     * @param speechs
     * Imports a list of speechs from an xml file to the model
     */
    static void readSpeechs(SpeechController *speechController, const QString fileName);

private:

    /**
     * @brief readNameElement
     * @param xmlReader
     * @return QString
     * Reads an attribute in an xml file
     */
    static QString readElement(QXmlStreamReader &xmlReader);

    /**
     * @brief clear
     * clears the file, only leaving the default group empty
     * careful this function assumes that you are absolutely sure it is applied to a speechs xml file
     */
    static void clear(SpeechController* speechController, const QString fileName);
};

#endif // SPEECHXMLPARSER_H
