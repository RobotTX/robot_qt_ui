#include "pathbuttongroup.h"
#include <QVBoxLayout>
#include "Model/paths.h"
#include "View/custompushbutton.h"
#include "View/stylesettings.h"

PathButtonGroup::PathButtonGroup(QWidget *_parent, QSharedPointer<Paths> _paths): QWidget(_parent), paths(_paths)
{
    layout = new QVBoxLayout(this);
    buttonGroup = new QButtonGroup(this);
    layout->setAlignment(Qt::AlignTop);
}

void PathButtonGroup::setGroupPaths(const QString groupName){
    qDebug() << "GroupsPathsButtonGroup::setGroupPaths called";
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
            groupButton->setIconSize(normal_icon_size);
        }
    }
}


