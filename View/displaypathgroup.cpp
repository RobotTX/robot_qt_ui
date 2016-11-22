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
#include "View/customlabel.h"
#include <QKeyEvent>
#include "View/pathpainter.h"

DisplayPathGroup::DisplayPathGroup(QWidget* _parent, MainWindow* _mainWindow, const QSharedPointer<Paths>& _paths):
    QWidget(_parent), mainWindow(_mainWindow), paths(_paths), lastCheckedButton("")
{
    /// to scroll the button group if there is a lot of paths
    CustomScrollArea* scrollArea = new CustomScrollArea(this, true);

    layout = new QVBoxLayout(this);
    QVBoxLayout* topLayout = new QVBoxLayout();

    /// 5 buttons displayed at the top
    actionButtons = new TopLeftMenu(this);

    initializeActionButtons();

    topLayout->addWidget(actionButtons);

    groupNameLabel = new CustomLabel("Group Name", this, true);
    topLayout->addWidget(groupNameLabel);
    layout->addLayout(topLayout);

    pathButtonGroup = new PathButtonGroup(this, paths);

    /// called when a button is clicked in the button group
    connect(pathButtonGroup->getButtonGroup(), SIGNAL(buttonClicked(QAbstractButton*)), this, SLOT(enableButtons(QAbstractButton*)));

    scrollArea->setWidget(pathButtonGroup);

    layout->addWidget(scrollArea);

    topLayout->setContentsMargins(0, 0, 10, 0);
    layout->setContentsMargins(0, 0, 0, 0);

    /// to handle double clicks
    foreach(QAbstractButton *button, pathButtonGroup->getButtonGroup()->buttons())
        connect(button, SIGNAL(doubleClick(QString)), _mainWindow, SLOT(doubleClickOnPath(QString)));

}

/// we reset the action buttons everytime we show the widget
void DisplayPathGroup::showEvent(QShowEvent *event){
    setPathsGroup(groupNameLabel->text());
    initializeActionButtons();
    updateDisplayedPath();
    /// for some reason the buttongroup sometimes becomes uncheckable after some operations, this just makes sure it does not happen
    setEnabled(true);
    pathButtonGroup->uncheck();
    QWidget::showEvent(event);
}

void DisplayPathGroup::initializeActionButtons(){
    actionButtons->enableAll(false);
    actionButtons->getPlusButton()->setEnabled(true);
    actionButtons->getMinusButton()->setCheckable(false);
    actionButtons->getEditButton()->setCheckable(false);
    actionButtons->getMapButton()->setCheckable(true);
    actionButtons->getMapButton()->setChecked(false);

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

        /// if the path is visible on the map the eye button is checked
        if(!mainWindow->getPathPainter()->getVisiblePath().compare(button->text()))
            actionButtons->getMapButton()->setChecked(true);

        /// updates the last checked button to be the one the user has just checked
        lastCheckedButton = button->text();

        /// updates the tooltips to explain the user what to do
        actionButtons->getMinusButton()->setEnabled(true);
        actionButtons->getMinusButton()->setToolTip("Click to remove the selected path");
        actionButtons->getMapButton()->setEnabled(true);
        actionButtons->getMapButton()->setToolTip("Click to display or hide the selected path on the map");
        actionButtons->getEditButton()->setEnabled(true);
        actionButtons->getEditButton()->setToolTip("Click to edit the selected path");
        actionButtons->getGoButton()->setEnabled(true);
        actionButtons->getGoButton()->setToolTip("Click to access the information of the selected path");

    } else {
        /// the path was already selected so we deselect it
        lastCheckedButton = "";
        initializeActionButtons();
        pathButtonGroup->uncheck();
        actionButtons->getMapButton()->setChecked(false);
    }
}

void DisplayPathGroup::resetMapButton(){
    actionButtons->getMapButton()->setChecked(false);
}

void DisplayPathGroup::setPathsGroup(const QString groupName){
    pathButtonGroup->deleteButtons();
    /// if the group of paths exists
    if(paths->getGroups()->find(groupName) != paths->getGroups()->end()){
        /// we iterate over it to create the buttons
        QSharedPointer<Paths::CollectionPaths> current_group = paths->getGroups()->value(groupName);
        QMapIterator<QString, QSharedPointer<Paths::Path>> it_paths(*current_group);
        int i(0);
        while(it_paths.hasNext()){
            it_paths.next();
            qDebug() << "found this path" << it_paths.key();
            CustomPushButton* groupButton = new CustomPushButton(it_paths.key(), this);
            groupButton->setIconSize(s_icon_size);

            /// if this path is displayed on the map we also add an icon to show it on the button
            if(!it_paths.key().compare(mainWindow->getPathPainter()->getVisiblePath()))
                groupButton->setIcon(QIcon(":/icons/eye.png"));
            else
                groupButton->setIcon(QIcon(":/icons/blank.png"));

            pathButtonGroup->getButtonGroup()->addButton(groupButton, i++);
            /// connects the button to the main window to handle double clicks on the button
            connect(groupButton, SIGNAL(doubleClick(QString)), mainWindow, SLOT(doubleClickOnPath(QString)));
            groupButton->setCheckable(true);
            pathButtonGroup->getLayout()->addWidget(groupButton);
        }
    }
}

/// sets the eye icon properly in front of the displayed path if such path exists
void DisplayPathGroup::updateDisplayedPath(){
    foreach(QAbstractButton* button, pathButtonGroup->getButtonGroup()->buttons()){
        if(!button->text().compare(mainWindow->getPathPainter()->getVisiblePath()))
            button->setIcon(QIcon(":/icons/eye.png"));
        else
            button->setIcon(QIcon(":/icons/blank.png"));
        button->setIconSize(s_icon_size);
    }
}

/// allows a user to delete a path with the delete key
void DisplayPathGroup::keyPressEvent(QKeyEvent *event){
    if(event->key() == Qt::Key_Delete){
        if(pathButtonGroup->getButtonGroup()->checkedButton())
            emit deletePath();
    }
}

void DisplayPathGroup::resizeEvent(QResizeEvent *event){
    QWidget* widget = static_cast<QWidget*>(parent());
    int maxWidth = widget->width() - 10;
    setMaximumWidth(maxWidth);
    QWidget::resizeEvent(event);
}
