#include "displaypathgroup.h"
#include "Model/paths.h"
#include "View/pathbuttongroup.h"
#include "View/customscrollarea.h"
#include "View/topleftmenu.h"
#include <QVBoxLayout>
#include <QMainWindow>
#include "View/custompushbutton.h"
#include "Controller/mainwindow.h"
#include "View/stylesettings.h"

DisplayPathGroup::DisplayPathGroup(QWidget* _parent, MainWindow* _mainWindow, const QSharedPointer<Paths>& _paths):
    QWidget(_parent), mainWindow(_mainWindow), paths(_paths), lastCheckedButton("")
{
    /// to scroll the button group if there is a lot of paths
    CustomScrollArea* scrollArea = new CustomScrollArea(this);

    layout = new QVBoxLayout(this);

    /// 5 buttons displayed at the top
    actionButtons = new TopLeftMenu(this);

    initializeActionButtons();

    layout->addWidget(actionButtons);

    pathButtonGroup = new PathButtonGroup(this, paths);

    /// called when a button is clicked in the button group
    connect(pathButtonGroup->getButtonGroup(), SIGNAL(buttonClicked(QAbstractButton*)), this, SLOT(enableButtons(QAbstractButton*)));

    scrollArea->setWidget(pathButtonGroup);

    layout->addWidget(scrollArea);

    /*setMaximumWidth(_parent->width()*4/10);
    setMinimumWidth(_parent->width()*4/10);*/
    //layout->setContentsMargins(0,0,0,0);

    /// to handle double clicks
    foreach(QAbstractButton *button, pathButtonGroup->getButtonGroup()->buttons())
        connect(button, SIGNAL(doubleClick(QString)), _mainWindow, SLOT(doubleClickOnPath(QString)));

}

/// we reset the action buttons everytime we show the widget
void DisplayPathGroup::showEvent(QShowEvent *event){
    initializeActionButtons();
    QWidget::showEvent(event);
}

void DisplayPathGroup::initializeActionButtons(){
    actionButtons->disableAll();
    actionButtons->getPlusButton()->setEnabled(true);
    actionButtons->getMinusButton()->setCheckable(false);
    actionButtons->getEditButton()->setCheckable(false);
    actionButtons->getMapButton()->setCheckable(true);

    actionButtons->getPlusButton()->setToolTip("Click to create a new path");
    actionButtons->getMinusButton()->setToolTip("Select a path and click here to remove it");
    actionButtons->getEditButton()->setToolTip("Select a path and click here to edit it");
    actionButtons->getGoButton()->setToolTip("Select a path and click here to access its information");
    actionButtons->getMapButton()->setToolTip("Select a path and click here to display or hide it on the map");
}

/// called when a button is clicked in the button group
/// if the button was already checked, the button is unchecked and the appropriate action buttons disabled
/// lastCheckButton is updated to keep track of the last checked button
void DisplayPathGroup::enableButtons(QAbstractButton *button){
    if(button->text().compare(lastCheckedButton)){
        lastCheckedButton = button->text();
        actionButtons->getMinusButton()->setEnabled(true);
        actionButtons->getMinusButton()->setToolTip("Click to remove the selected path");
        actionButtons->getMapButton()->setEnabled(true);
        actionButtons->getMapButton()->setToolTip("Click to display or hide the selected path on the map");
        actionButtons->getEditButton()->setEnabled(true);
        actionButtons->getEditButton()->setToolTip("Click to edit the selected path");
        actionButtons->getGoButton()->setEnabled(true);
        actionButtons->getGoButton()->setToolTip("Click to access the information of the selected path");
    } else { /// the path was already selected so we deselect it
        lastCheckedButton = "";
        initializeActionButtons();
        pathButtonGroup->uncheck();
    }
}

void DisplayPathGroup::resetMapButton(){
    qDebug() << "DisplayPathGroup::resetMapButton called";
    actionButtons->getMapButton()->setChecked(false);
}

void DisplayPathGroup::setPathsGroup(const QString groupName){
    qDebug() << "GroupsPathsButtonGroup::setGroupPaths called";
    pathButtonGroup->deleteButtons();
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
            pathButtonGroup->getButtonGroup()->addButton(groupButton, i++);
            connect(groupButton, SIGNAL(doubleClick(QString)), mainWindow, SLOT(doubleClickOnPath(QString)));
            groupButton->setCheckable(true);
            pathButtonGroup->getLayout()->addWidget(groupButton);
            groupButton->setIconSize(normal_icon_size);
        }
    }
}
