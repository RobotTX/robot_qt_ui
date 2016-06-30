#include "xmlparser.h"
#include "points.h"
#include "robot.h"
#include "robots.h"
#include "group.h"
#include "point.h"
#include "View/robotview.h"
#include <QFile>
#include <QXmlStreamWriter>
#include <QDebug>

XMLParser::XMLParser(const QString filename, QGraphicsItem* const& _parent): parent(_parent)
{
    file = new QFile(filename);
}

XMLParser::~XMLParser(){
    delete file;
}

void XMLParser::save(const Points& points) const {
    try {

        file->open(QIODevice::WriteOnly);

        QXmlStreamWriter xmlWriter(file);
        xmlWriter.setAutoFormatting(true);
        xmlWriter.writeStartDocument();

        xmlWriter.writeStartElement("points");

        for(int i = 0; i < points.getGroups().size(); i++){
            xmlWriter.writeStartElement("group");
            xmlWriter.writeTextElement("name", points.getGroups()[i]->getName());
            // for each point of the group
            for(int j = 0; j < points.getGroups()[i]->getPoints().size(); j++){
                xmlWriter.writeStartElement("point");
                std::shared_ptr<Point> currPoint = points.getGroups()[i]->getPoints()[j];
                xmlWriter.writeTextElement("name", currPoint->getName());
                xmlWriter.writeTextElement("x", QString::number(currPoint->getPosition().getX())); // the x attribute of a point object or node
                xmlWriter.writeTextElement("y", QString::number(currPoint->getPosition().getY())); // the y attribute of a point object or node
                xmlWriter.writeTextElement("displayed", QString::number(currPoint->isDisplayed()));
                xmlWriter.writeEndElement();
            }
            xmlWriter.writeEndElement();
        }

        xmlWriter.writeEndElement();

        file->close();

    } catch(std::exception e) {
        std::cout << e.what() << std::endl;
    }
}


QVector<QString> XMLParser::readRobots(Robots& robots){

    QXmlStreamReader xmlReader;
    QVector<QString> robotsNames;

    try {

        file->open(QFile::ReadOnly | QFile::Text);

        xmlReader.setDevice(file);
        xmlReader.readNext();

        while(!xmlReader.atEnd()){
            if(xmlReader.isStartElement()){
                if(xmlReader.name() == "robots"){
                    xmlReader.readNext();
                }
                else if(xmlReader.name() == "robot"){
                    std::shared_ptr<Robot> robot = std::make_shared<Robot>(Robot());
                    while(!xmlReader.atEnd()){
                        if(xmlReader.isEndElement()){
                            xmlReader.readNext();
                            break;
                        }
                        else if(xmlReader.isCharacters()){
                            xmlReader.readNext();
                        }
                        else if(xmlReader.isStartElement()){
                            if(xmlReader.name() == "name"){
                                QString robotName = readNameElement(xmlReader);
                                robot->setName(robotName);
                                robotsNames.push_back(robotName);
                            }
                            else if(xmlReader.name() == "IP"){
                                robot->setIp(readIPElement(xmlReader));
                                robots.add(new RobotView(robot, parent));
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

        file->close();
    }

    catch(std::exception e) {
        qDebug() << "here ";
        qDebug() << e.what();
    }

    return robotsNames;
}

QString XMLParser::readNameElement(QXmlStreamReader& xmlReader){
    QString nameElement("");
    while(!xmlReader.atEnd()){
        if(xmlReader.isEndElement()){
            xmlReader.readNext();
            break;
        }
        else if(xmlReader.isStartElement()){
            nameElement = xmlReader.readElementText();
            xmlReader.readNext();
            break;
        }
        else if(xmlReader.isCharacters()){
            xmlReader.readNext();
        } else {
            xmlReader.readNext();
        }
    }
    return nameElement;
}

QString XMLParser::readIPElement(QXmlStreamReader& xmlReader){
    QString IPElement("");
    while(!xmlReader.atEnd()){
        if(xmlReader.isEndElement()){
            xmlReader.readNext();
            break;
        }
        else if(xmlReader.isStartElement()){
            IPElement = xmlReader.readElementText();
            xmlReader.readNext();
            break;
        }
        else if(xmlReader.isCharacters()){
            xmlReader.readNext();
        } else {
            xmlReader.readNext();
        }
    }
    return IPElement;
}

float XMLParser::readCoordinateElement(QXmlStreamReader &xmlReader){
    float coordinate(0.0);
    while(!xmlReader.atEnd()){
        if(xmlReader.isEndElement()){
            xmlReader.readNext();
            break;
        }
        else if(xmlReader.isStartElement()){
            coordinate = xmlReader.readElementText().toFloat();
            xmlReader.readNext();
            break;
        }
        else if(xmlReader.isCharacters()){
            xmlReader.readNext();
        } else {
            xmlReader.readNext();
        }
    }
    return coordinate;
}

void XMLParser::readPoints(std::shared_ptr<Points>& points){
    QXmlStreamReader xmlReader;

    try {

        file->open(QFile::ReadOnly | QFile::Text);

        xmlReader.setDevice(file);
        xmlReader.readNext();

        while(!xmlReader.atEnd()){
            if(xmlReader.isStartElement()){
                if(xmlReader.name() == "points"){
                    xmlReader.readNext();
                }
                else if(xmlReader.name() == "group"){
                    Group* group = new Group();
                    while(!xmlReader.atEnd()){
                        if(xmlReader.isEndElement()){
                            xmlReader.readNext();
                            break;
                        }
                        else if(xmlReader.isCharacters()){
                            xmlReader.readNext();
                        }
                        else if(xmlReader.isStartElement()){
                            if(xmlReader.name() == "name"){
                                group->setName(readNameElement(xmlReader));
                            }
                            else if(xmlReader.name() == "point"){
                                Point point;
                                float x(0.0);
                                float y(0.0);
                                QString name;
                                bool displayed;
                                point.setPosition(x, y);
                                xmlReader.readNext();
                                while(!xmlReader.atEnd()){

                                    if(xmlReader.isStartElement()){
                                        if(xmlReader.name() == "x"){
                                            x = readCoordinateElement(xmlReader);
                                            xmlReader.readNext();
                                        }
                                        else if(xmlReader.name() == "y"){
                                            y = readCoordinateElement(xmlReader);
                                            xmlReader.readNext();
                                        }
                                        else if(xmlReader.name() == "name"){
                                            name = readNameElement(xmlReader);
                                            point.setName(name);
                                            xmlReader.readNext();
                                        }
                                        else if(xmlReader.name() == "displayed"){
                                            displayed = readDisplayedElement(xmlReader);
                                            point.setDisplayed(displayed);
                                            xmlReader.readNext();
                                        }
                                        else {
                                            xmlReader.readNext();
                                        }
                                    }
                                    else if(xmlReader.isEndElement()){
                                        xmlReader.readNext();
                                        break;
                                    }
                                    else xmlReader.readNext();
                                }

                                point.setPosition(x, y);
                                group->addPoint(std::make_shared<Point> (Point(point)));
                            }

                            xmlReader.readNext();
                        } else {
                            xmlReader.readNext();
                        }
                    }
                    points->addGroup(*group);
                }



            } else {
                xmlReader.readNext();
            }
        }

        file->close();
    }

    catch(std::exception e) {
        qDebug() << "here ";
        qDebug() << e.what();
    }
}

bool XMLParser::readDisplayedElement(QXmlStreamReader &xmlReader){
    bool displayed(false);
    while(!xmlReader.atEnd()){
        if(xmlReader.isEndElement()){
            xmlReader.readNext();
            break;
        }
        else if(xmlReader.isStartElement()){
            displayed = xmlReader.readElementText().toInt();
            xmlReader.readNext();
            break;
        }
        else if(xmlReader.isCharacters()){
            xmlReader.readNext();
        } else {
            xmlReader.readNext();
        }
    }
    return displayed;
}

void XMLParser::clear(void){
    try {
        qDebug() << "ok";
        file->open(QIODevice::WriteOnly);

        QXmlStreamWriter xmlWriter(file);
        xmlWriter.setAutoFormatting(true);
        xmlWriter.writeStartDocument();

        xmlWriter.writeStartElement("points");

        xmlWriter.writeStartElement("group");
        xmlWriter.writeTextElement("name", "no group");

        xmlWriter.writeEndElement();
        xmlWriter.writeEndElement();

        file->close();

    } catch(std::exception e) {
        std::cout << e.what() << std::endl;
    }
}
