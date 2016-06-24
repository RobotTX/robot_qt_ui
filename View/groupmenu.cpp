#include "groupmenu.h"
#include "Model/points.h"
#include "Model/group.h"
#include "Model/point.h"
#include <QVBoxLayout>
#include <QListWidget>

GroupMenu::GroupMenu(const Points& points, bool _editPoint, QWidget* parent): QWidget(parent), editPoint(_editPoint)
{
    //setStyleSheet("background-color: transparent;");
    layout = new QVBoxLayout(this);


    widgetsList = new QListWidget(this);

    /// to remove the border (very ugly atm)
    ///widgetsList->setFrameShape(QFrame::NoFrame);

    if(!editPoint){

        for(int i = 0; i < points.getGroups().size()-1; i++)
            /// we add the names of the groups to the list
            widgetsList->addItem(points.getGroups().at(i)->getName());
            //widgetsList->item(i)->

        /// for the last group we just want to show the points and not "no group"
        for(int j = 0; j < points.getGroups().at(points.getGroups().size()-1)->getPoints().size(); j++){
            std::cout << points.getGroups().at(points.getGroups().size()-1)->getPoints().size() << std::endl;
            //std::cout << points.getGroups().at(points.getGroups().size()-1)->getPoints().at(j)->getName() << std::endl;
            std::shared_ptr<Point> curr_point = points.getGroups().at(points.getGroups().size()-1)->getPoints().at(j);
            widgetsList->addItem(curr_point->getName() + " (" + QString::number(curr_point->getPosition().getX()) + ", " +
                                 QString::number(curr_point->getPosition().getY()) + ")");
        }
    } else {
        for(int i = 0; i < points.getGroups().size(); i++)
            /// we add the names of the groups to the list
            widgetsList->addItem(points.getGroups().at(i)->getName());
    }

    layout->setAlignment(Qt::AlignTop);
    layout->addWidget(widgetsList);
}

void GroupMenu::updateList(const Points& points){
    widgetsList->clear();

    if(!editPoint){

        for(int i = 0; i < points.getGroups().size()-1; i++)
            widgetsList->addItem(points.getGroups().at(i)->getName());
        /// for the last group we just want to show the points and not "no group"
        for(int j = 0; j < points.getGroups().at(points.getGroups().size()-1)->getPoints().size(); j++){
            std::cout << points.getGroups().at(points.getGroups().size()-1)->getPoints().size() << std::endl;
            //std::cout << points.getGroups().at(points.getGroups().size()-1)->getPoints().at(j)->getName() << std::endl;
            std::shared_ptr<Point> curr_point = points.getGroups().at(points.getGroups().size()-1)->getPoints().at(j);
            widgetsList->addItem(curr_point->getName() + " (" + QString::number(curr_point->getPosition().getX()) + ", " +
                                 QString::number(curr_point->getPosition().getY()) + ")");
        }
    } else {
        for(int i = 0; i < points.getGroups().size(); i++)
            /// we add the names of the groups to the list
            widgetsList->addItem(points.getGroups().at(i)->getName());
    }
}

void GroupMenu::displayReverse(void){
    std::vector<QString> listGroups;
    for(int i = 0; i < widgetsList->count(); i++)
        listGroups.push_back(widgetsList->item(i)->text());

    widgetsList->clear();

    for(size_t i = 0; i < listGroups.size(); i++)
        widgetsList->addItem(listGroups.at(listGroups.size()-1-i));
}
