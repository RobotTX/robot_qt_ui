#include "helper.h"
#include <QTime>
#include <QCoreApplication>
#include <QDebug>
#include <QMessageBox>
#include <QFile>
#include <QFileInfo>
#include <QStringList>
#include <QDir>

namespace Helper {

    namespace Convert {

        Position pixelCoordToRobotCoord(const Position positionInPixels, double originX, double originY, double resolution, int height){
            float xInRobotCoordinates = (positionInPixels.getX()) * resolution + originX;
            float yInRobotCoordinates = (-positionInPixels.getY() + height) * resolution + originY;
            return Position(xInRobotCoordinates, yInRobotCoordinates);
        }

        Position robotCoordToPixelCoord(const Position positionInRobotCoordinates, double originX, double originY, double resolution, int height){
            float xInPixelCoordinates = (-originX+ positionInRobotCoordinates.getX())/resolution;
            float yInPixelCoordinates = height - (-originY + positionInRobotCoordinates.getY()) / resolution;
            return Position(xInPixelCoordinates, yInPixelCoordinates);
        }
    }

    namespace Date {

        bool isLater(const QStringList &date, const QStringList &otherDate){
            /// to ensure that we don't crash even if one date is not complete
            if(date.size() < otherDate.size())
                return false;
            else if(date.size() > otherDate.size())
                return true;
            else {
                for(int i = 0; i < date.size(); i++){
                    if(date.at(i).toInt() > otherDate.at(i).toInt())
                        return true;
                    else if(date.at(i).toInt() < otherDate.at(i).toInt())
                        return false;
                }
                return false;
            }
        }
    }

    namespace File {

        QPair<QPair<QString, QString>, QStringList> getPathFromFile(const QString robotName){
            /// QPair<QPair<groupName, pathName>, date>
            QPair<QPair<QString, QString>, QStringList> pathInfo;
            QFile fileInfo(QDir::currentPath() + QDir::separator() + "robots_paths" + QDir::separator() + robotName + "_path");
            if(fileInfo.open(QIODevice::ReadWrite)){
                QRegExp regex("[-\n%]");
                QString content = fileInfo.readAll();
                QStringList l = content.split(regex, QString::SkipEmptyParts);
                qDebug() << "path QStringlist" << l;
                if(l.size() == 8){
                    for(int i = 0; i < 6; i++)
                        pathInfo.second.push_back(l.at(i));
                    pathInfo.first.first = l.at(6);
                    pathInfo.first.second = l.at(7);
                }
            }
            fileInfo.close();
            return pathInfo;
        }

        void updateHomeFile(const QString robotName, const Position& robot_home_position, const QStringList date){
            qDebug() << "updatehomefile" << robotName << date.size();
            QFile fileWriteHome(QDir::currentPath() + QDir::separator() + "robots_homes" + QDir::separator() + robotName);
            if(fileWriteHome.open(QIODevice::ReadWrite)){
                QTextStream out(&fileWriteHome);
                out << robot_home_position.getX() << " " << robot_home_position.getY() << "\n";
                for(int i = 0; i < date.size()-1; i++)
                    out << date.at(i) << "-";
                out << date.at(date.size()-1);
                fileWriteHome.close();
            } else
                qDebug() << "could not update the home of" << robotName;
        }

        QPair<Position, QStringList> getHomeFromFile(const QString robotName){
            /// retrieves the home point of the robot if the robot has one
            QFile fileInfo(QDir::currentPath() + QDir::separator() + "robots_homes" + QDir::separator() + robotName);
            Position p;
            QStringList dateLastModification;
            if(fileInfo.open(QIODevice::ReadWrite)){
                QRegExp regex("[-\n ]");
                QString content = fileInfo.readAll();
                if(!content.compare(""))
                    content = "0-0-1970-01-01-00-00-00";
                content.replace("\n", " ");
                QStringList l = content.split(regex, QString::SkipEmptyParts);
                qDebug() << "app list" << l;
                if(l.size() > 0){
                    p.setX(l.at(0).toDouble());
                    p.setY(l.at(1).toDouble());
                    for(int i = 2; i < l.size(); i++)
                        dateLastModification.push_back(l.at(i));
                }
            }
            fileInfo.close();
            return QPair<Position, QStringList> (p, dateLastModification);
        }
    }

    namespace Prompt {
        /**
         * @brief MainWindow::openConfirmMessage
         * @param text
         * @return int
         * prompts the user for confirmation
         */
        int openConfirmMessage(const QString text){
            QMessageBox msgBox;
            msgBox.setText(text);
            msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
            msgBox.setDefaultButton(QMessageBox::Cancel);
            return msgBox.exec();
        }

    }

    QString formatName(const QString name) {
        qDebug() << "GroupsPathsWidget::formatName called" << name;

        QString ret("");
        QStringList nameStrList = name.split(" ", QString::SkipEmptyParts);
        for(int i = 0; i < nameStrList.size(); i++){
            if(i > 0)
                ret += " ";
            ret += nameStrList.at(i);
        }
        if(name.size() > 0 && name.at(name.size()-1) == ' ')
            ret += " ";
        return ret;

    }
}
