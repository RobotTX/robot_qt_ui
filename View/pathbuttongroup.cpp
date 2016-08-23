#include "pathbuttongroup.h"
#include <QVBoxLayout>
#include "Model/paths.h"
#include "View/colors.h"
#include "View/doubleclickablebutton.h"

PathButtonGroup::PathButtonGroup(QWidget *_parent, QSharedPointer<Paths> _paths): QWidget(_parent), paths(_paths)
{
    layout = new QVBoxLayout(this);
    buttonGroup = new QButtonGroup(this);
    layout->setAlignment(Qt::AlignTop);
    BUTTON_SIZE = parentWidget()->size()/2;

}

void PathButtonGroup::setGroupPaths(const QString groupName){
    qDebug() << "GroupsPathsButtonGroup::createButtons called";
    /// if the group of paths exists
    if(paths->getGroups()->find(groupName) != paths->getGroups()->end()){
        /// we iterate over it to create the buttons
        QSharedPointer<Paths::CollectionPaths> current_group = paths->getGroups()->value(groupName);
        QMapIterator<QString, QSharedPointer<Paths::Path>> it_paths(*current_group);
        while(it_paths.hasNext()){
            it_paths.next();

            DoubleClickableButton* groupButton = new DoubleClickableButton(it_paths.key(), this);
            groupButton->setAutoDefault(true);
            groupButton->setFlat(true);
            groupButton->setStyleSheet("QPushButton {color: "+text_color+";text-align:left;border: 4px; padding: 10px;}QPushButton:hover{background-color: "+button_hover_color+";}QPushButton:checked{background-color: "+button_checked_color+";}");

            buttonGroup->addButton(groupButton);
            layout->addWidget(groupButton);

            groupButton->setIconSize(BUTTON_SIZE);
        }
    }
}
