#include "xmlparser.h"
#include <QXmlStreamWriter>
#include <QDebug>
#include "Model/Point/points.h"
#include "Model/Point/point.h"

XMLParser::XMLParser() {}

XMLParser::~XMLParser(){
}

void XMLParser::save(const Points& points, const QString fileName) {
  /*  try {
        QFile file(fileName);
        file.open(QIODevice::WriteOnly);

        QXmlStreamWriter xmlWriter(&file);
        xmlWriter.setAutoFormatting(true);
        xmlWriter.writeStartDocument();

        xmlWriter.writeStartElement("points");

        QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points.getGroups()));
        /// For each group
        while (i.hasNext()) {
            i.next();

            if(i.key().compare(PATH_GROUP_NAME) && i.key().compare(TMP_GROUP_NAME)){
                xmlWriter.writeStartElement("group");
                xmlWriter.writeTextElement("name", i.key());

                /// For each point of the group
                for(int j = 0; j < i.value()->size(); j++){
                    xmlWriter.writeStartElement("point");
                    xmlWriter.writeTextElement("name", i.value()->at(j)->getPoint()->getName());
                    xmlWriter.writeTextElement("x", QString::number(i.value()->at(j)->getPoint()->getPosition().getX()));
                    xmlWriter.writeTextElement("y", QString::number(i.value()->at(j)->getPoint()->getPosition().getY()));
                    xmlWriter.writeTextElement("displayed", QString::number(i.value()->at(j)->isVisible()));
                    xmlWriter.writeEndElement();
                }
                xmlWriter.writeEndElement();
            }
        }
        xmlWriter.writeEndElement();
        file.close();

    } catch(std::exception e) {
        std::cout << e.what() << std::endl;
    }*/
}


QString XMLParser::readNameElement(QXmlStreamReader& xmlReader){
    QString nameElement("");
    while(!xmlReader.atEnd()){
        if(xmlReader.isEndElement()){
            xmlReader.readNext();
            break;
        } else if(xmlReader.isStartElement()){
            nameElement = xmlReader.readElementText();
            xmlReader.readNext();
            break;
        } else if(xmlReader.isCharacters())
            xmlReader.readNext();
        else
            xmlReader.readNext();
    }
    return nameElement;
}

QString XMLParser::readIPElement(QXmlStreamReader& xmlReader){
    QString IPElement("");
    while(!xmlReader.atEnd()){
        if(xmlReader.isEndElement()){
            xmlReader.readNext();
            break;
        } else if(xmlReader.isStartElement()){
            IPElement = xmlReader.readElementText();
            xmlReader.readNext();
            break;
        } else if(xmlReader.isCharacters())
            xmlReader.readNext();
        else
            xmlReader.readNext();
    }
    return IPElement;
}

float XMLParser::readCoordinateElement(QXmlStreamReader &xmlReader){
    float coordinate(0.0);
    while(!xmlReader.atEnd()){
        if(xmlReader.isEndElement()){
            xmlReader.readNext();
            break;
        } else if(xmlReader.isStartElement()){
            coordinate = xmlReader.readElementText().toFloat();
            xmlReader.readNext();
            break;
        } else if(xmlReader.isCharacters())
            xmlReader.readNext();
        else
            xmlReader.readNext();
    }
    return coordinate;
}

void XMLParser::readPoints(Points* points, const QString fileName){
    QXmlStreamReader xmlReader;

    try {
        QFile file(fileName);

        file.open(QFile::ReadOnly | QFile::Text);

        xmlReader.setDevice(&file);
        xmlReader.readNext();

        while(!xmlReader.atEnd()){
            if(xmlReader.isStartElement()){
                if(xmlReader.name() == "points"){
                    xmlReader.readNext();
                } else if(xmlReader.name() == "group"){
                    QString groupName("");
                    while(!xmlReader.atEnd()){
                        if(xmlReader.isEndElement()){
                            xmlReader.readNext();
                            break;
                        } else if(xmlReader.isCharacters()){
                            xmlReader.readNext();
                        } else if(xmlReader.isStartElement()){
                            if(xmlReader.name() == "name"){
                                groupName = readNameElement(xmlReader);
                                points->addGroup(groupName);
                            } else if(xmlReader.name() == "point"){
                                double x(0.0);
                                double y(0.0);
                                QString name;
                                bool displayed;
                                xmlReader.readNext();
                                while(!xmlReader.atEnd()){

                                    if(xmlReader.isStartElement()){
                                        if(xmlReader.name() == "x"){
                                            x = readCoordinateElement(xmlReader);
                                            xmlReader.readNext();
                                        } else if(xmlReader.name() == "y"){
                                            y = readCoordinateElement(xmlReader);
                                            xmlReader.readNext();
                                        } else if(xmlReader.name() == "name"){
                                            name = readNameElement(xmlReader);
                                            xmlReader.readNext();
                                        } else if(xmlReader.name() == "displayed"){
                                            displayed = readDisplayedElement(xmlReader);
                                            xmlReader.readNext();
                                        } else {
                                            xmlReader.readNext();
                                        }
                                    } else if(xmlReader.isEndElement()){
                                        xmlReader.readNext();
                                        break;
                                    } else xmlReader.readNext();
                                }
                                points->addPoint(groupName, name, x, y, displayed);
                            }
                            xmlReader.readNext();
                        } else {
                            xmlReader.readNext();
                        }
                    }
                }
            } else {
                xmlReader.readNext();
            }
        }
        file.close();
    }

    catch(std::exception e) {
        qDebug() << "Exception in XMLParser::readPoints :" << e.what();
    }
}

bool XMLParser::readDisplayedElement(QXmlStreamReader &xmlReader){
    bool displayed(false);
    while(!xmlReader.atEnd()){
        if(xmlReader.isEndElement()){
            xmlReader.readNext();
            break;
        } else if(xmlReader.isStartElement()){
            displayed = xmlReader.readElementText().toInt();
            xmlReader.readNext();
            break;
        } else if(xmlReader.isCharacters())
            xmlReader.readNext();
        else
            xmlReader.readNext();
    }
    return displayed;
}

/// resets the file, only writting an empty default group
void XMLParser::clear(const QString fileName){
    try {
        QFile file(fileName);
        file.open(QIODevice::WriteOnly);

        QXmlStreamWriter xmlWriter(&file);
        xmlWriter.setAutoFormatting(true);
        xmlWriter.writeStartDocument();

        xmlWriter.writeStartElement("points");

        xmlWriter.writeStartElement("group");
        xmlWriter.writeTextElement("name", NO_GROUP_NAME);

        xmlWriter.writeEndElement();
        xmlWriter.writeEndElement();

        file.close();

    } catch(std::exception e) {
        qDebug() << e.what();
    }
}
