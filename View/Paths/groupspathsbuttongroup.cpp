#include "View/Paths/groupspathsbuttongroup.h"
#include <QVBoxLayout>
#include "View/Other/custompushbutton.h"
#include "Model/Paths/paths.h"
#include "View/Other/stylesettings.h"
#include "View/Other/customlineedit.h"

GroupsPathsButtonGroup::GroupsPathsButtonGroup(QWidget *_parent): QWidget(_parent)
{
    layout = new QVBoxLayout(this);
    btnGroup = new QButtonGroup(this);
    layout->setAlignment(Qt::AlignTop);

    modifyEdit = new CustomLineEdit(this);
    modifyEdit->setFixedWidth(1.29 * modifyEdit->width());
    modifyEdit->hide();

    /// we are going to make this widget visible when a user wants to modify the name of a group
    layout->addWidget(modifyEdit);
    layout->setContentsMargins(0, 0, 10, 0);
}

void GroupsPathsButtonGroup::updateButtons(QSharedPointer<Paths> const& paths){
    /// Remove buttons
    QList<QAbstractButton*> listBtn = btnGroup->buttons();
    for(int i = 0; i < listBtn.size(); i++){
        btnGroup->removeButton(listBtn.at(i));
        layout->removeWidget(listBtn.at(i));
        delete listBtn.at(i);
    }

    /// to give a different id to each button
    int i(0);
    QMapIterator<QString, QSharedPointer<Paths::CollectionPaths>> it_paths_groups(*(paths->getGroups()));
    while(it_paths_groups.hasNext()){
        it_paths_groups.next();
        CustomPushButton* groupButton = new CustomPushButton(QIcon(":/icons/folder.png"), it_paths_groups.key(), this, CustomPushButton::ButtonType::LEFT_MENU, "left", true);
        btnGroup->addButton(groupButton, i++);
        layout->addWidget(groupButton);
        groupButton->setIconSize(s_icon_size);
    }

    emit updateConnectionsRequest();
}

/// to sort of reset the state of the QButtonGroup (like if the user just got on this page)
void GroupsPathsButtonGroup::uncheck(){
    btnGroup->setExclusive(false);
    if(btnGroup->checkedButton())
        btnGroup->checkedButton()->setChecked(false);
    btnGroup->setExclusive(true);
}

/// to enable or disable the QButtonGroup depending on what the user is doing
void GroupsPathsButtonGroup::setEnabledGroup(const bool enable){
    qDebug() << "GroupButtonGroup::setEnabled called";
    foreach(QAbstractButton* button, btnGroup->buttons())
        button->setEnabled(enable);
}

void GroupsPathsButtonGroup::resizeEvent(QResizeEvent *event){
    QWidget* widget = static_cast<QWidget*>(parent()->parent()->parent());
    int maxWidth = widget->width() - 10;
    setMaximumWidth(maxWidth);
    QWidget::resizeEvent(event);
}
