#include "groupspathswidget.h"
#include "Controller/mainwindow.h"
#include "View/topleftmenu.h"
#include <QDebug>
#include <QLabel>
#include <QVBoxLayout>
#include "Model/point.h"
#include "View/groupspathsbuttongroup.h"
#include "View/pathbuttongroup.h"
#include "View/customizedlineedit.h"
#include <QAbstractButton>
#include <QPushButton>
#include "View/customscrollarea.h"
#include <QHBoxLayout>

GroupsPathsWidget::GroupsPathsWidget(MainWindow* _parent, const QSharedPointer<Paths> &_paths): QWidget(_parent), paths(_paths), lastCheckedButton("")
{
    scrollArea = new CustomScrollArea(this);

    layout = new QVBoxLayout(this);

    QVBoxLayout* downLayout = new QVBoxLayout();

    groupNameLabel = new QLabel("New group's name : ", this);
    groupNameLabel->hide();
    groupNameEdit = new CustomizedLineEdit(this);
    groupNameEdit->hide();

    downLayout->addWidget(groupNameEdit);
    downLayout->addWidget(groupNameLabel);

    /// to modify the names of groups of paths
    modifyEdit = new CustomizedLineEdit(this);
    modifyEdit->setFixedWidth(1.29*modifyEdit->width());
    modifyEdit->hide();

    actionButtons = new TopLeftMenu(this);

    /// initializes the appropriate tooltips
    actionButtons->getEditButton()->setToolTip("Select a group of paths and click here to modify it");
    actionButtons->getMinusButton()->setToolTip("Select a group of paths and click here to delete it");
    actionButtons->getGoButton()->setToolTip("Select a group of paths and click here to display its paths");
    actionButtons->getPlusButton()->setToolTip("Click here to add a new group of paths");

    /// disables buttons until one of the group of paths is clicked
    actionButtons->getEditButton()->setEnabled(false);
    actionButtons->getMinusButton()->setEnabled(false);
    actionButtons->getMapButton()->setEnabled(false);
    actionButtons->getGoButton()->setEnabled(false);

    layout->addWidget(actionButtons);

    buttonGroup = new GroupsPathsButtonGroup(_parent, paths);

    scrollArea->setWidget(buttonGroup);
    downLayout->addWidget(scrollArea);

    layout->addWidget(buttonGroup);
    /// to enable the buttons
    connect(buttonGroup->getButtonGroup(), SIGNAL(buttonClicked(QAbstractButton*)), this, SLOT(enableButtons(QAbstractButton*)));    

    creationLayout = new QHBoxLayout();
    saveButton = new QPushButton("Save", this);
    saveButton->setAutoDefault(true);
    saveButton->hide();
    saveButton->setEnabled(false);
    cancelButton = new QPushButton("Cancel", this);
    cancelButton->setAutoDefault(true);
    cancelButton->hide();
    creationLayout->addWidget(cancelButton);
    creationLayout->addWidget(saveButton);

    downLayout->setAlignment(Qt::AlignBottom);
    layout->setAlignment(Qt::AlignTop);
    layout->addLayout(downLayout);
    hide();
}

void GroupsPathsWidget::enableButtons(QAbstractButton* button){

    if(!button->text().compare(lastCheckedButton)){
        buttonGroup->uncheck();
        lastCheckedButton = "";
        disableButtons();

    } else {
        qDebug() << "GroupsPathsWidget::enableButtons enabling buttons";
        lastCheckedButton = button->text();
        /*
        groupButtonGroup->setEditedGroupName(button);
        groupButtonGroup->getLayout()->removeWidget(groupButtonGroup->getModifyEdit());
        groupButtonGroup->getLayout()->addWidget(groupButtonGroup->getModifyEdit());
        */
        disableButtons();
        /// enables the minus button
        actionButtons->getMinusButton()->setEnabled(true);

        actionButtons->getMinusButton()->setToolTip("Click to remove the selected group");


        /// enables the eye button
        actionButtons->getGoButton()->setEnabled(true);
        actionButtons->getGoButton()->setToolTip("Click to display the paths of this group");

        /// enables the edit button
        actionButtons->getEditButton()->setEnabled(true);
        actionButtons->getEditButton()->setToolTip("Click to modify the selected group");
    }
}

void GroupsPathsWidget::disableButtons(){
    qDebug() << "GroupsPathsWidget disableButtons called";
    actionButtons->getMinusButton()->setEnabled(false);
    actionButtons->getMinusButton()->setToolTip("Select a group of paths and click here to delete it");

    actionButtons->getGoButton()->setEnabled(false);
    actionButtons->getGoButton()->setToolTip("Select a group of paths and click here to display its paths");

    actionButtons->getEditButton()->setEnabled(false);
    actionButtons->getEditButton()->setToolTip("Select a group of paths and click here to modify it");
}
