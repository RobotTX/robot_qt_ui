#include "View/groupspathsbuttongroup.h"
#include <QVBoxLayout>
#include "View/custompushbutton.h"
#include "Model/paths.h"
#include "View/stylesettings.h"
#include "View/customlineedit.h"

GroupsPathsButtonGroup::GroupsPathsButtonGroup(QWidget *_parent, const QSharedPointer<Paths> &_paths): QWidget(_parent), paths(_paths)
{
    layout = new QVBoxLayout(this);
    buttonGroup = new QButtonGroup(this);
    layout->setAlignment(Qt::AlignTop);

    createButtons();

    modifyEdit = new CustomLineEdit(this);
    modifyEdit->setFixedWidth(1.29 * modifyEdit->width());
    modifyEdit->hide();

    /// we are going to make this widget visible when a user wants to modify the name of a group
    layout->addWidget(modifyEdit);
    layout->setContentsMargins(0, 0, 10, 0);
}

void GroupsPathsButtonGroup::createButtons(){
    /// to give a different id to each button
    int i(0);
    QMapIterator<QString, QSharedPointer<Paths::CollectionPaths>> it_paths_groups(*(paths->getGroups()));
    while(it_paths_groups.hasNext()){
        it_paths_groups.next();
        CustomPushButton* groupButton = new CustomPushButton(QIcon(":/icons/folder.png"), it_paths_groups.key(), this, false, CustomPushButton::ButtonType::LEFT_MENU, "left", true);
        buttonGroup->addButton(groupButton, i++);
        layout->addWidget(groupButton);
        groupButton->setIconSize(s_icon_size);
    }
}

/// to sort of reset the state of the QButtonGroup (like if the user just got on this page)
void GroupsPathsButtonGroup::uncheck(){
    buttonGroup->setExclusive(false);
    if(buttonGroup->checkedButton())
        buttonGroup->checkedButton()->setChecked(false);
    buttonGroup->setExclusive(true);
}

/// delete the buttons so they can be reconstructed (for example after a group of paths has been added)
void GroupsPathsButtonGroup::deleteButtons(){
    qDebug() << "GroupPathsButtonGroup::deleteButtons called";
    layout->removeWidget(modifyEdit);
    while(QLayoutItem* item = layout->takeAt(0)){
        if(item){
            if(QWidget* button = item->widget()){
                delete button;
                button = 0;
            }
            delete item;
            item = 0;
        } else
            qDebug() << "GroupButtonGroup::deleButtons item is null";
    }
    layout->addWidget(modifyEdit);
}

/// to enable or disable the QButtonGroup depending on what the user is doing
void GroupsPathsButtonGroup::setEnabledGroup(const bool enable){
    qDebug() << "GroupButtonGroup::setEnabled called";
    foreach(QAbstractButton* button, buttonGroup->buttons())
        button->setEnabled(enable);
}

void GroupsPathsButtonGroup::resizeEvent(QResizeEvent *event){
    QWidget* widget = static_cast<QWidget*>(parent()->parent()->parent());
    int maxWidth = widget->width() - 10;
    setMaximumWidth(maxWidth);
    QWidget::resizeEvent(event);
}
