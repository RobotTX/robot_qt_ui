#include "speechxmlparser.h"
#include <QXmlStreamWriter>
#include <QDebug>
#include <QPointer>
#include "Helper/helper.h"
#include "Controller/Speech/speechcontroller.h"
#include "Model/Speech/speechs.h"
#include "Model/Speech/speechgroup.h"
#include "Model/Speech/speech.h"

SpeechXMLParser::SpeechXMLParser(){
}

void SpeechXMLParser::save(SpeechController *speechController, const QString fileName) {
    try {
        qDebug() << "SpeechXMLParser::save the speechs in" << fileName;
        QFile file(fileName);
        file.open(QIODevice::Truncate | QIODevice::WriteOnly);

        QXmlStreamWriter xmlWriter(&file);
        xmlWriter.setAutoFormatting(true);
        xmlWriter.writeStartDocument();

        xmlWriter.writeStartElement("speechs");

        /// We write the "No Group" first
        xmlWriter.writeStartElement("group");
        xmlWriter.writeTextElement("name", NO_GROUP_NAME);

        /// For each speech of the group
        if(speechController->getSpeechs()->getGroups().contains(NO_GROUP_NAME)){
            for(int j = 0; j < speechController->getSpeechs()->getGroups().value(NO_GROUP_NAME)->getSpeechVector().size(); j++){
                xmlWriter.writeStartElement("speech");
                xmlWriter.writeTextElement("name", speechController->getSpeechs()->getGroups().value(NO_GROUP_NAME)->getSpeechVector().at(j)->getName());
                xmlWriter.writeTextElement("tts", speechController->getSpeechs()->getGroups().value(NO_GROUP_NAME)->getSpeechVector().at(j)->getTts());
                xmlWriter.writeEndElement();
            }
            xmlWriter.writeEndElement();
        }

        QMapIterator<QString, QPointer<SpeechGroup>> i(speechController->getSpeechs()->getGroups());
        /// For each group except "No Group"
        while (i.hasNext()) {
            i.next();

            if(i.key().compare(NO_GROUP_NAME) != 0){
                xmlWriter.writeStartElement("group");
                xmlWriter.writeTextElement("name", i.key());

                /// For each speech of the group
                for(int j = 0; j < i.value()->getSpeechVector().size(); j++){
                    xmlWriter.writeStartElement("speech");
                    xmlWriter.writeTextElement("name", i.value()->getSpeechVector().at(j)->getName());
                    xmlWriter.writeTextElement("tts", i.value()->getSpeechVector().at(j)->getTts());
                    xmlWriter.writeEndElement();
                }
                xmlWriter.writeEndElement();
            }
        }
        xmlWriter.writeEndElement();
        file.close();

    } catch(std::exception e) {
        qDebug() << "SpeechXMLParser::save" << e.what();
    }
}

QString SpeechXMLParser::readElement(QXmlStreamReader& xmlReader){
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

void SpeechXMLParser::readSpeechs(SpeechController* speechController, const QString fileName){
    QXmlStreamReader xmlReader;

    QFile file(fileName);

    if(file.exists() && file.open(QFile::ReadWrite | QFile::Text)){

        xmlReader.setDevice(&file);
        xmlReader.readNext();

        while(!xmlReader.atEnd()){
            if(xmlReader.isStartElement()){
                if(xmlReader.name() == "speechs"){
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
                                speechController->addGroup(groupName, false);
                            } else if(xmlReader.name() == "speech"){
                                QString name;
                                QString tts;
                                xmlReader.readNext();
                                while(!xmlReader.atEnd()){

                                    if(xmlReader.isStartElement()){
                                        if(xmlReader.name() == "tts"){
                                            tts = readElement(xmlReader);
                                            xmlReader.readNext();
                                        } else if(xmlReader.name() == "name"){
                                            name = readElement(xmlReader);
                                            xmlReader.readNext();
                                        } else {
                                            xmlReader.readNext();
                                        }
                                    } else if(xmlReader.isEndElement()){
                                        xmlReader.readNext();
                                        break;
                                    } else xmlReader.readNext();
                                }
                                speechController->addSpeech(name, groupName, tts, false);
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
        if(speechController->getSpeechs()->getGroups().empty())
            clear(speechController, fileName);
    } else {
        clear(speechController, fileName);
    }
}

/// resets the file, only writting an empty default group
void SpeechXMLParser::clear(SpeechController* speechController, const QString fileName){

    try {

        QFile file(fileName);
        file.open(QIODevice::WriteOnly);

        QXmlStreamWriter xmlWriter(&file);
        xmlWriter.setAutoFormatting(true);
        xmlWriter.writeStartDocument();

        xmlWriter.writeStartElement("speechs");

        xmlWriter.writeStartElement("group");
        xmlWriter.writeTextElement("name", NO_GROUP_NAME);
        speechController->addGroup(NO_GROUP_NAME);

        xmlWriter.writeEndElement();
        xmlWriter.writeEndElement();

        file.close();

    } catch(std::exception e) {
        qDebug() << e.what();
    }
}
