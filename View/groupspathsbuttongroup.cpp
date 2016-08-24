#include "View/groupspathsbuttongroup.h"
#include <QVBoxLayout>
#include "View/custompushbutton.h"
#include "Model/paths.h"
#include "View/colors.h"

GroupsPathsButtonGroup::GroupsPathsButtonGroup(QWidget *_parent, QSharedPointer<Paths> _paths): QWidget(_parent), paths(_paths)
{
    layout = new QVBoxLayout(this);
    buttonGroup = new QButtonGroup(this);
    layout->setAlignment(Qt::AlignTop);
    BUTTON_SIZE = parentWidget()->size()/20;
    createButtons();
}

void GroupsPathsButtonGroup::createButtons(){
    qDebug() << "GroupsPathsButtonGroup::createButtons called";

    QMapIterator<QString, QSharedPointer<Paths::CollectionPaths>> it_paths_groups(*(paths->getGroups()));
    while(it_paths_groups.hasNext()){
        it_paths_groups.next();

        CustomPushButton* groupButton = new CustomPushButton(it_paths_groups.key(), this);
        //groupButton->setAutoDefault(true);

        buttonGroup->addButton(groupButton);
        layout->addWidget(groupButton);
        groupButton->setIcon(QIcon(":/icons/folder.png"));

        groupButton->setIconSize(BUTTON_SIZE);
    }
}
