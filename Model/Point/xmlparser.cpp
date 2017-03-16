#include "xmlparser.h"
#include <QXmlStreamWriter>
#include <QDebug>
#include <QPointer>
#include "Controller/Point/pointcontroller.h"
#include "Model/Point/points.h"
#include "Model/Point/pointgroup.h"
#include "Model/Point/point.h"

XMLParser::XMLParser() {}

XMLParser::~XMLParser(){
}

void XMLParser::save(PointController *pointController, const QString fileName) {
    try {
        qDebug() << "XMLParser::save the points in" << fileName;
        QFile file(fileName);
        file.open(QIODevice::Truncate | QIODevice::WriteOnly);

        QXmlStreamWriter xmlWriter(&file);
        xmlWriter.setAutoFormatting(true);
        xmlWriter.writeStartDocument();

        xmlWriter.writeStartElement("points");


        QMapIterator<QString, QPointer<PointGroup>> i(pointController->getPoints()->getGroups());
        /// For each group
        while (i.hasNext()) {
            i.next();

            xmlWriter.writeStartElement("group");
            xmlWriter.writeTextElement("name", i.key());

            /// For each point of the group
            for(int j = 0; j < i.value()->getPointVector().size(); j++){
                xmlWriter.writeStartElement("point");
                xmlWriter.writeTextElement("name", i.value()->getPointVector().at(j)->getName());
                xmlWriter.writeTextElement("x", QString::number(i.value()->getPointVector().at(j)->getX()));
                xmlWriter.writeTextElement("y", QString::number(i.value()->getPointVector().at(j)->getY()));
                xmlWriter.writeTextElement("displayed", QString::number(i.value()->getPointVector().at(j)->isVisible()));
                xmlWriter.writeEndElement();
            }
            xmlWriter.writeEndElement();
        }
        xmlWriter.writeEndElement();
        file.close();

    } catch(std::exception e) {
        qDebug() << "XMLParser::save" << e.what();
    }
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

void XMLParser::readPoints(PointController* pointController, const QString fileName){
    QXmlStreamReader xmlReader;

    try {
        QFile file(fileName);

        file.open(QFile::ReadWrite | QFile::Text);

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
                                pointController->addGroup(groupName, false);
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
                                pointController->addPoint(name, groupName, x, y, "", "", displayed, false);
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
