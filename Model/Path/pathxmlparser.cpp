#include "pathxmlparser.h"
#include <QXmlStreamWriter>
#include <QDebug>
#include <QPointer>
#include <QFile>
#include "Helper/helper.h"
#include "Controller/Path/pathcontroller.h"
#include "Model/Path/paths.h"
#include "Model/Path/pathgroup.h"
#include "Model/Path/path.h"
#include "Model/Path/pathpoint.h"
#include "Model/Point/point.h"

PathXMLParser::PathXMLParser(){
}

void PathXMLParser::save(PathController *pathController, const QString fileName) {

    try {
        qDebug() << "PathXMLParser::save the paths in" << fileName;
        QFile file(fileName);
        file.open(QIODevice::Truncate | QIODevice::WriteOnly);

        QXmlStreamWriter xmlWriter(&file);
        xmlWriter.setAutoFormatting(true);
        xmlWriter.writeStartDocument();

        xmlWriter.writeStartElement("paths");

        /// We write the "No Group" first
        xmlWriter.writeStartElement("group");
        xmlWriter.writeTextElement("name", NO_GROUP_NAME);

        //if(pathController->getPaths()){
        QMapIterator<QString, QPointer<Path>> l(pathController->getPaths()->getGroups().value(NO_GROUP_NAME)->getPaths());
        /// For each path
        while (l.hasNext()) {
            l.next();

            xmlWriter.writeStartElement("path");
            xmlWriter.writeTextElement("name", l.key());

            /// For each point of the path
            for(int k = 0; k < l.value()->getPathPointVector().size(); k++){
                xmlWriter.writeStartElement("pathpoint");
                xmlWriter.writeTextElement("name", l.value()->getPathPointVector().at(k)->getPoint()->getName());
                xmlWriter.writeTextElement("x", QString::number(l.value()->getPathPointVector().at(k)->getPoint()->getPos().x()));
                xmlWriter.writeTextElement("y", QString::number(l.value()->getPathPointVector().at(k)->getPoint()->getPos().y()));
                xmlWriter.writeTextElement("waittime", QString::number(l.value()->getPathPointVector().at(k)->getWaitTime()));
                xmlWriter.writeEndElement();
            }
            xmlWriter.writeEndElement();
        }
        xmlWriter.writeEndElement();

        QMapIterator<QString, QPointer<PathGroup>> i(pathController->getPaths()->getGroups());
        /// For each group except "No Group"
        while (i.hasNext()) {
            i.next();

            if(i.key().compare(NO_GROUP_NAME) != 0){

                xmlWriter.writeStartElement("group");
                xmlWriter.writeTextElement("name", i.key());


                QMapIterator<QString, QPointer<Path>> j(i.value()->getPaths());
                /// For each path
                while (j.hasNext()) {
                    j.next();

                    xmlWriter.writeStartElement("path");
                    xmlWriter.writeTextElement("name", j.key());


                    /// For each point of the path
                    for(int k = 0; k < j.value()->getPathPointVector().size(); k++){
                        xmlWriter.writeStartElement("pathpoint");
                        xmlWriter.writeTextElement("name", j.value()->getPathPointVector().at(k)->getPoint()->getName());
                        xmlWriter.writeTextElement("x", QString::number(j.value()->getPathPointVector().at(k)->getPoint()->getPos().x()));
                        xmlWriter.writeTextElement("y", QString::number(j.value()->getPathPointVector().at(k)->getPoint()->getPos().y()));
                        xmlWriter.writeTextElement("waittime", QString::number(j.value()->getPathPointVector().at(k)->getWaitTime()));
                        xmlWriter.writeEndElement();
                    }
                    xmlWriter.writeEndElement();
                }
                xmlWriter.writeEndElement();
            }
        }
        xmlWriter.writeEndElement();
        file.close();

    } catch(std::exception e) {
        qDebug() << "PathXMLParser::save" << e.what();
    }
}

QString PathXMLParser::readNameElement(QXmlStreamReader& xmlReader){
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

float PathXMLParser::readCoordinateElement(QXmlStreamReader &xmlReader){
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

void PathXMLParser::readPaths(PathController *pathController, const QString fileName){

    QXmlStreamReader xmlReader;

    /// TODO if cannot open -> create and call clear to create group

    QFile file(fileName);

    if(file.exists() && file.open(QFile::ReadWrite | QFile::Text)){

        xmlReader.setDevice(&file);
        xmlReader.readNext();

        while(!xmlReader.atEnd()){
            if(xmlReader.isStartElement()){
                if(xmlReader.name() == "paths"){
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
                                pathController->addGroup(groupName, false);
                            } else if(xmlReader.name() == "path"){
                                QString pathName("");
                                while(!xmlReader.atEnd()){
                                    if(xmlReader.isEndElement()){
                                        xmlReader.readNext();
                                        break;
                                    } else if(xmlReader.isCharacters()){
                                        xmlReader.readNext();
                                    } else if(xmlReader.isStartElement()){
                                        if(xmlReader.name() == "name"){
                                            pathName = readNameElement(xmlReader);
                                            pathController->addPath(groupName, pathName, false);
                                        } else if(xmlReader.name() == "pathpoint"){
                                            double x(0.0);
                                            double y(0.0);
                                            QString name;
                                            int waitTime(0);
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
                                                    } else if(xmlReader.name() == "waittime"){
                                                        waitTime = readWaitTimeElement(xmlReader);
                                                        xmlReader.readNext();
                                                    } else {
                                                        xmlReader.readNext();
                                                    }
                                                } else if(xmlReader.isEndElement()){
                                                    xmlReader.readNext();
                                                    break;
                                                } else xmlReader.readNext();
                                            }
                                            pathController->addPathPoint(groupName, pathName, name, x, y, waitTime, false);

                                            xmlReader.readNext();
                                        } else {
                                            xmlReader.readNext();
                                        }
                                    }
                                }
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
        if(pathController->getPaths()->getGroups().empty())
            clear(pathController, fileName);
    } else {
        clear(pathController, fileName);
    }
}

int PathXMLParser::readWaitTimeElement(QXmlStreamReader &xmlReader){
    int waitTime(0);
    while(!xmlReader.atEnd()){
        if(xmlReader.isEndElement()){
            xmlReader.readNext();
            break;
        } else if(xmlReader.isStartElement()){
            waitTime = xmlReader.readElementText().toInt();
            xmlReader.readNext();
            break;
        } else if(xmlReader.isCharacters())
            xmlReader.readNext();
        else
            xmlReader.readNext();
    }
    return waitTime;
}

/// resets the file, only writting an empty default group
void PathXMLParser::clear(PathController *pathController, const QString fileName){

    try {
        QFile file(fileName);
        file.resize(0);
        file.open(QIODevice::WriteOnly);

        QXmlStreamWriter xmlWriter(&file);
        xmlWriter.setAutoFormatting(true);
        xmlWriter.writeStartDocument();

        xmlWriter.writeStartElement("paths");

        xmlWriter.writeStartElement("group");
        xmlWriter.writeTextElement("name", NO_GROUP_NAME);
        pathController->addGroup(NO_GROUP_NAME);

        xmlWriter.writeEndElement();
        xmlWriter.writeEndElement();

        file.close();

    } catch(std::exception e) {
        qDebug() << e.what();
    }
}
