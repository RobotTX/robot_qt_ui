#include "xmlparser.h"
#include <QXmlStreamWriter>
#include <QDebug>
#include <QPointer>
#include "Helper/helper.h"
#include "Controller/Point/pointcontroller.h"
#include "Model/Point/points.h"
#include "Model/Point/pointgroup.h"
#include "Model/Point/point.h"

XMLParser::XMLParser(){
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

        /// We write the "No Group" first
        xmlWriter.writeStartElement("group");
        xmlWriter.writeTextElement("name", NO_GROUP_NAME);

        /// For each point of the group
        if(pointController->getPoints()->getGroups().contains(NO_GROUP_NAME)){
            for(int j = 0; j < pointController->getPoints()->getGroups().value(NO_GROUP_NAME)->getPointVector().size(); j++){
                xmlWriter.writeStartElement("point");
                xmlWriter.writeTextElement("name", pointController->getPoints()->getGroups().value(NO_GROUP_NAME)->getPointVector().at(j)->getName());
                xmlWriter.writeTextElement("x", QString::number(pointController->getPoints()->getGroups().value(NO_GROUP_NAME)->getPointVector().at(j)->getPos().x()));
                xmlWriter.writeTextElement("y", QString::number(pointController->getPoints()->getGroups().value(NO_GROUP_NAME)->getPointVector().at(j)->getPos().y()));
                xmlWriter.writeTextElement("displayed", QString::number(pointController->getPoints()->getGroups().value(NO_GROUP_NAME)->getPointVector().at(j)->isVisible()));
                xmlWriter.writeTextElement("home", QString::number(pointController->getPoints()->getGroups().value(NO_GROUP_NAME)->getPointVector().at(j)->isHome()));
                xmlWriter.writeTextElement("orientation", QString::number(pointController->getPoints()->getGroups().value(NO_GROUP_NAME)->getPointVector().at(j)->getOrientation()));
                xmlWriter.writeEndElement();
            }
            xmlWriter.writeEndElement();
        }

        QMapIterator<QString, QPointer<PointGroup>> i(pointController->getPoints()->getGroups());
        /// For each group except "No Group"
        while (i.hasNext()) {
            i.next();

            if(i.key().compare(NO_GROUP_NAME) != 0){
                xmlWriter.writeStartElement("group");
                xmlWriter.writeTextElement("name", i.key());

                /// For each point of the group
                for(int j = 0; j < i.value()->getPointVector().size(); j++){
                    xmlWriter.writeStartElement("point");
                    xmlWriter.writeTextElement("name", i.value()->getPointVector().at(j)->getName());
                    xmlWriter.writeTextElement("x", QString::number(i.value()->getPointVector().at(j)->getPos().x()));
                    xmlWriter.writeTextElement("y", QString::number(i.value()->getPointVector().at(j)->getPos().y()));
                    xmlWriter.writeTextElement("displayed", QString::number(i.value()->getPointVector().at(j)->isVisible()));
                    xmlWriter.writeTextElement("home", QString::number(i.value()->getPointVector().at(j)->isHome()));
                    xmlWriter.writeTextElement("orientation", QString::number(i.value()->getPointVector().at(j)->getOrientation()));
                    xmlWriter.writeEndElement();
                }
                xmlWriter.writeEndElement();
            }
        }
        xmlWriter.writeEndElement();
        file.close();

    } catch(std::exception e) {
        qDebug() << "XMLParser::save" << e.what();
    }
}

QString XMLParser::readElement(QXmlStreamReader& xmlReader){
    QString element("");
    while(!xmlReader.atEnd()){
        if(xmlReader.isEndElement()){
            xmlReader.readNext();
            break;
        } else if(xmlReader.isStartElement()){
            element = xmlReader.readElementText();
            xmlReader.readNext();
            break;
        } else if(xmlReader.isCharacters())
            xmlReader.readNext();
        else
            xmlReader.readNext();
    }
    return element;
}

void XMLParser::readPoints(PointController* pointController, const QString fileName){
    QXmlStreamReader xmlReader;

    QFile file(fileName);

    if(file.exists() && file.open(QFile::ReadWrite | QFile::Text)){

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
                                groupName = readElement(xmlReader);
                                pointController->addGroup(groupName, false);
                            } else if(xmlReader.name() == "point"){
                                double x(0.0);
                                double y(0.0);
                                QString name;
                                bool displayed(true);
                                bool home(false);
                                int orientation(0);
                                xmlReader.readNext();
                                while(!xmlReader.atEnd()){

                                    if(xmlReader.isStartElement()){
                                        if(xmlReader.name() == "x"){
                                            x = readElement(xmlReader).toDouble();
                                            xmlReader.readNext();
                                        } else if(xmlReader.name() == "y"){
                                            y = readElement(xmlReader).toDouble();
                                            xmlReader.readNext();
                                        } else if(xmlReader.name() == "name"){
                                            name = readElement(xmlReader);
                                            xmlReader.readNext();
                                        } else if(xmlReader.name() == "displayed"){
                                            displayed = readElement(xmlReader).toInt();
                                            xmlReader.readNext();
                                        } else if(xmlReader.name() == "home"){
                                            home = readElement(xmlReader).toInt();
                                            xmlReader.readNext();
                                        } else if(xmlReader.name() == "orientation"){
                                            orientation = readElement(xmlReader).toInt();
                                            xmlReader.readNext();
                                        } else {
                                            xmlReader.readNext();
                                        }
                                    } else if(xmlReader.isEndElement()){
                                        xmlReader.readNext();
                                        break;
                                    } else xmlReader.readNext();
                                }
                                pointController->addPoint(name, groupName, x, y, "", "", displayed, home, orientation, false);
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
        /// in case the file exists but is empty we add the no group to it
        if(pointController->getPoints()->getGroups().empty())
            clear(pointController, fileName);
    } else {
        clear(pointController, fileName);
    }
}

/// resets the file, only writting an empty default group
void XMLParser::clear(PointController* pointController, const QString fileName){

    try {

        QFile file(fileName);
        file.open(QIODevice::WriteOnly);

        QXmlStreamWriter xmlWriter(&file);
        xmlWriter.setAutoFormatting(true);
        xmlWriter.writeStartDocument();

        xmlWriter.writeStartElement("points");

        xmlWriter.writeStartElement("group");
        xmlWriter.writeTextElement("name", NO_GROUP_NAME);
        pointController->addGroup(NO_GROUP_NAME);

        xmlWriter.writeEndElement();
        xmlWriter.writeEndElement();

        file.close();

    } catch(std::exception e) {
        qDebug() << e.what();
    }
}
