#include "View/groupspathsbuttongroup.h"
#include <QVBoxLayout>
#include "View/custompushbutton.h"
#include "Model/paths.h"
#include "View/colors.h"
#include "View/customizedlineedit.h"

GroupsPathsButtonGroup::GroupsPathsButtonGroup(QWidget *_parent, QSharedPointer<Paths> _paths): QWidget(_parent), paths(_paths)
{
    layout = new QVBoxLayout(this);
    buttonGroup = new QButtonGroup(this);
    layout->setAlignment(Qt::AlignTop);
    BUTTON_SIZE = parentWidget()->size()/20;
    createButtons();

    /// to modify the name of a group
    modifyEdit = new CustomizedLineEdit(this);
    modifyEdit->setFixedWidth(1.29*modifyEdit->width());
    modifyEdit->hide();

    /// we are going to make this widget visible when a user wants to modify a group
    layout->addWidget(modifyEdit);
}

void GroupsPathsButtonGroup::createButtons(){
    qDebug() << "GroupsPathsButtonGroup::createButtons called";

    QMapIterator<QString, QSharedPointer<Paths::CollectionPaths>> it_paths_groups(*(paths->getGroups()));
    while(it_paths_groups.hasNext()){
        it_paths_groups.next();
        CustomPushButton* groupButton = new CustomPushButton(it_paths_groups.key(), this, true);

        buttonGroup->addButton(groupButton);
        layout->addWidget(groupButton);
        groupButton->setIcon(QIcon(":/icons/folder.png"));
        groupButton->setIconSize(BUTTON_SIZE);
    }
}

void GroupsPathsButtonGroup::uncheck(){
    buttonGroup->setExclusive(false);
    if(buttonGroup->checkedButton())
        buttonGroup->checkedButton()->setChecked(false);
    buttonGroup->setExclusive(true);
}

void GroupsPathsButtonGroup::deleteButtons(){
    qDebug() << "GroupButtonGroup::deleteButtons called";
    layout->removeWidget(modifyEdit);
    while(QLayoutItem* item = layout->takeAt(0)){
        if(item){
            if(QWidget* button = item->widget())
                delete button;
        } else {
            qDebug() << "oops";
        }
    }
    layout->addWidget(modifyEdit);
}
