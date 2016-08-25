#include "pathbuttongroup.h"
#include <QVBoxLayout>
#include "Model/paths.h"
#include "View/custompushbutton.h"

PathButtonGroup::PathButtonGroup(QWidget *_parent, QSharedPointer<Paths> _paths): QWidget(_parent), paths(_paths)
{
    layout = new QVBoxLayout(this);
    buttonGroup = new QButtonGroup(this);
    layout->setAlignment(Qt::AlignTop);
    BUTTON_SIZE = parentWidget()->size()/2;
}

void PathButtonGroup::setGroupPaths(const QString groupName){
    qDebug() << "GroupsPathsButtonGroup::setGroupPaths called";
    deleteButtons();
    /// if the group of paths exists
    if(paths->getGroups()->find(groupName) != paths->getGroups()->end()){
        qDebug() << "found" << groupName;
        /// we iterate over it to create the buttons
        QSharedPointer<Paths::CollectionPaths> current_group = paths->getGroups()->value(groupName);
        QMapIterator<QString, QSharedPointer<Paths::Path>> it_paths(*current_group);
        int i(0);
        while(it_paths.hasNext()){
            it_paths.next();
            qDebug() << "found this path" << it_paths.key();
            CustomPushButton* groupButton = new CustomPushButton(it_paths.key(), this);
            //groupButton->setAutoDefault(true);
            buttonGroup->addButton(groupButton, i++);
            groupButton->setCheckable(true);
            layout->addWidget(groupButton);
            groupButton->setIconSize(BUTTON_SIZE);
        }
    }
}

void PathButtonGroup::deleteButtons(void){
    qDebug() << "PathButtonGroup::deleteButtons called";
    while(QLayoutItem* item = layout->takeAt(0)){
        if(QWidget* button = item->widget())
            delete button;
    }
}

void PathButtonGroup::uncheck(){
    buttonGroup->setExclusive(false);
    if(buttonGroup->checkedButton())
        buttonGroup->checkedButton()->setChecked(false);
    buttonGroup->setExclusive(true);
}
