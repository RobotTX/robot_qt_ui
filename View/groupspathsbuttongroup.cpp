#include "View/groupspathsbuttongroup.h"
#include <QVBoxLayout>
#include "View/doubleclickablebutton.h"
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

        DoubleClickableButton* groupButton = new DoubleClickableButton(it_paths_groups.key(), this);
        groupButton->setAutoDefault(true);
        groupButton->setFlat(true);
        groupButton->setStyleSheet("QPushButton {color: "+text_color+";text-align:left;border: 4px; padding: 10px;}QPushButton:hover{background-color: "+button_hover_color+";}QPushButton:checked{background-color: "+button_checked_color+";}");

        buttonGroup->addButton(groupButton);
        layout->addWidget(groupButton);
        groupButton->setIcon(QIcon(":/icons/folder.png"));

        groupButton->setIconSize(BUTTON_SIZE);
    }
}
