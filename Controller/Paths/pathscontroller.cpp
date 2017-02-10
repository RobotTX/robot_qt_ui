#include "pathscontroller.h"
#include <QLineEdit>
#include <QDir>
#include "Controller/TopLayout/toplayoutcontroller.h"
#include "Model/Paths/paths.h"
#include "Model/Points/points.h"
#include "View/Paths/pathcreationwidget.h"
#include "View/Paths/displaypathgroup.h"
#include "View/Paths/displayselectedpath.h"
#include "View/Paths/groupspathswidget.h"
#include "View/Paths/pathpainter.h"
#include "View/LeftMenu/topleftmenu.h"
#include "View/Points/pointview.h"
#include "View/Paths/pathbuttongroup.h"
#include "View/Paths/pathpointcreationwidget.h"
#include "View/Other/customlineedit.h"
#include "View/Other/customlabel.h"
#include "View/Other/stylesettings.h"

PathsController::PathsController(MainWindow *mainWindow, const QSharedPointer<Points> points): QObject(mainWindow)
{
    paths = QSharedPointer<Paths>(new Paths());

    pathPainter = new PathPainter(mainWindow, points);

    initializePaths();

    /// Menu which displays the groups of paths
    groupsPathsWidget = new GroupsPathsWidget(mainWindow, paths);
    groupsPathsWidget->hide();

    displaySelectedPath = new DisplaySelectedPath(mainWindow, paths);
    displaySelectedPath->hide();
    connect(displaySelectedPath, SIGNAL(displayPath(QString, QString, bool)), this, SLOT(displayPathSlot(QString, QString, bool)));

    connect(groupsPathsWidget->getActionButtons()->getGoButton(), SIGNAL(clicked()), mainWindow, SLOT(displayGroupPaths()));
    connect(groupsPathsWidget->getActionButtons()->getEditButton(), SIGNAL(clicked()), this, SLOT(editGroupPaths()));
    connect(groupsPathsWidget->getActionButtons()->getPlusButton(), SIGNAL(clicked()), mainWindow, SLOT(createGroupPaths()));
    connect(groupsPathsWidget->getActionButtons()->getMinusButton(), SIGNAL(clicked()), mainWindow, SLOT(deleteGroupPaths()));
    /// to delete a group with the delete key
    connect(groupsPathsWidget, SIGNAL(deleteGroup()), mainWindow, SLOT(deleteGroupPaths()));

    pathGroup = new DisplayPathGroup(mainWindow, paths);
    pathGroup->hide();

    connect(pathGroup->getActionButtons()->getGoButton(), SIGNAL(clicked()), mainWindow, SLOT(displayPath()));
    connect(pathGroup->getActionButtons()->getPlusButton(), SIGNAL(clicked()), mainWindow, SLOT(createPath()));
    connect(pathGroup->getActionButtons()->getMinusButton(), SIGNAL(clicked()), mainWindow, SLOT(deletePath()));
    connect(pathGroup->getActionButtons()->getMapButton(), SIGNAL(clicked(bool)), mainWindow, SLOT(displayPathOnMap(bool)));
    connect(pathGroup->getActionButtons()->getEditButton(), SIGNAL(clicked()), mainWindow, SLOT(editPath()));
    /// to delete a path with the delete key
    connect(pathGroup, SIGNAL(deletePath()), mainWindow, SLOT(deletePath()));

    connect(pathGroup->getPathButtonGroup()->getButtonGroup(), SIGNAL(buttonToggled(int, bool)), pathGroup, SLOT(resetMapButton()));

    pathCreationWidget = new PathCreationWidget(mainWindow, points, paths, false);
    connect(pathCreationWidget, SIGNAL(addPathPoint(QString, double, double, int)), pathPainter, SLOT(addPathPointSlot(QString, double, double, int)));
    connect(pathCreationWidget, SIGNAL(deletePathPoint(int)), pathPainter, SLOT(deletePathPointSlot(int)));
    connect(pathCreationWidget, SIGNAL(orderPathPointChanged(int, int)), pathPainter, SLOT(orderPathPointChangedSlot(int, int)));
    connect(pathCreationWidget, SIGNAL(resetPath()), pathPainter, SLOT(resetPathSlot()));
    connect(pathCreationWidget, SIGNAL(setMessage(QString, QString)), mainWindow->getTopLayoutController(), SLOT(setLabel(QString, QString)));
    connect(pathCreationWidget, SIGNAL(actionChanged(int, QString)), pathPainter, SLOT(actionChangedSlot(int, QString)));
    connect(pathCreationWidget, SIGNAL(editPathPoint(int, QString, double, double)), pathPainter, SLOT(editPathPointSlot(int, QString, double, double)));

    pathCreationWidget->hide();

    connect(pathGroup, SIGNAL(checkEyeButton(QString)), this, SLOT(checkEyeButtonSlot(QString)));
    /// to handle double clicks
    foreach(QAbstractButton *button, pathGroup->getPathButtonGroup()->getButtonGroup()->buttons())
        connect(button, SIGNAL(doubleClick(QString)), mainWindow, SLOT(doubleClickOnPath(QString)));

    connect(pathGroup, SIGNAL(updateDisplayedPath()), this, SLOT(updateDisplayedPath()));
    connect(pathGroup, SIGNAL(setPathsGroup(QString)), this, SLOT(setPathsGroup(QString)));

    connect(mainWindow, SIGNAL(updatePathPainter(bool)), pathPainter, SLOT(updatePathPainterSlot(bool)));

    connect(pathCreationWidget, SIGNAL(editTmpPathPoint(int, QString, double, double)), mainWindow, SLOT(editTmpPathPointSlot(int, QString, double, double)));

    connect(mainWindow, SIGNAL(updatePathPainterPointView()), pathPainter, SLOT(updatePathPainterPointViewSlot()));

    connect(pathCreationWidget, SIGNAL(saveEditPathPoint()), mainWindow, SLOT(saveEditPathPointSlot()));

    connect(pathCreationWidget, SIGNAL(cancelEditPathPoint()), mainWindow, SLOT(cancelEditPathPointSlot()));

    connect(pathCreationWidget, SIGNAL(savePath()), mainWindow, SLOT(savePathSlot()));

    connect(mainWindow, SIGNAL(resetPath()), pathPainter, SLOT(resetPathSlot()));

    connect(mainWindow, SIGNAL(resetPathCreationWidget()), pathCreationWidget, SLOT(resetWidget()));

    connect(groupsPathsWidget, SIGNAL(newPathGroup(QString)), mainWindow, SLOT(saveGroupPaths(QString)));
    connect(groupsPathsWidget, SIGNAL(messageCreationGroup(QString, QString)), mainWindow->getTopLayoutController(), SLOT(setLabel(QString,QString)));
    connect(groupsPathsWidget, SIGNAL(modifiedGroup(QString)), mainWindow, SLOT(modifyGroupPathsWithEnter(QString)));

    connect(pathCreationWidget->getCancelButton(), SIGNAL(clicked()), mainWindow, SLOT(cancelNoRobotPathSlot()));

    connect(pathCreationWidget, SIGNAL(codeEditPath(int)), mainWindow, SLOT(setMessageNoRobotPath(int)));

}

void PathsController::initializePaths(){
    deserializePaths(QDir::currentPath() + QDir::separator() + "paths.dat");
}

void PathsController::serializePaths(const QString fileName){
    QFile pathFile(fileName);
    pathFile.resize(0);
    pathFile.open(QIODevice::ReadWrite);
    QDataStream out(&pathFile);
    out << *paths;
    pathFile.close();
}

void PathsController::deserializePaths(const QString fileName){
    QFile pathFile(fileName);
    pathFile.open(QIODevice::ReadWrite);
    QDataStream in(&pathFile);
    Paths tmpPaths;
    in >> tmpPaths;
    pathFile.close();
    paths->setGroups(tmpPaths.getGroups());
}

void PathsController::enableSaveEditButton(const bool enable){
    static_cast<PathPointCreationWidget*> (pathCreationWidget->getPathPointList()->
                                       itemWidget(pathCreationWidget->getPathPointList()->currentItem()))
                                       -> getSaveEditBtn()->setEnabled(enable);
}

void PathsController::hideGroupCreationWidgets() const {
    groupsPathsWidget->getSaveButton()->hide();
    groupsPathsWidget->getCancelButton()->hide();
    groupsPathsWidget->getGroupNameEdit()->hide();
    groupsPathsWidget->getGroupNameLabel()->hide();
}

void PathsController::enableGroupsPathsWidgetPlusButtonOnly() const {
    groupsPathsWidget->disableButtons();
    groupsPathsWidget->getActionButtons()->getPlusButton()->setEnabled(true);
    groupsPathsWidget->getActionButtons()->getPlusButton()->setToolTip("Click here to add a new group of paths");
}

void PathsController::editPath(const QString group, const QString path){
    showPathCreationWidget();
    pathPainter->setOldPath(pathPainter->getCurrentPath());

    pathCreationWidget->resetWidget();
    pathCreationWidget->setCurrentPathName(path);
    pathCreationWidget->setCurrentGroupName(group);

    setLastPathChecked(path);
    setVisiblePath(path);

    bool foundFlag(false);
    pathCreationWidget->updatePath(paths->getPath(group, path, foundFlag));
}

QString PathsController::editPath(){
    pathCreationWidget->show();
    pathPainter->setOldPath(pathPainter->getCurrentPath());

    const QString pathName = pathGroup->getPathButtonGroup()->getButtonGroup()->checkedButton()->text();
    const QString groupName = pathGroup->getGroupNameLabel()->text();

    /// stop displaying the currently displayed path if it exists
    pathCreationWidget->resetWidget();

    /// to be able to know if the path name is different from the old one
    qDebug() << "setting the current path name to" << pathName;
    pathCreationWidget->setCurrentPathName(pathName);
    pathCreationWidget->setCurrentGroupName(groupName);

    pathGroup->setLastCheckedButton(pathName);
    setVisiblePath(pathName);

    qDebug() << "will edit" <<  groupName << pathName;
    bool foundFlag(false);
    pathCreationWidget->updatePath(paths->getPath(pathGroup->getGroupNameLabel()->text(),
                                                  pathGroup->getPathButtonGroup()->getButtonGroup()->checkedButton()->text(), foundFlag));
    return pathName;
}

bool PathsController::deletePath(){
    bool already_existed(false);
    pathPainter->setPathDeleted(false);
    pathPainter->setOldPath(pathPainter->getCurrentPath());
    const QString group = pathCreationWidget->getCurrentGroupName();
    const QString path = pathCreationWidget->getNameEdit()->text().simplified();
    if(!pathCreationWidget->getCurrentPathName().isEmpty())
        already_existed = deletePath(group, pathCreationWidget->getCurrentPathName());

    createPath(group, path);

    for(int i = 0; i < pathPainter->getCurrentPath().size(); i++)
        addPathPoint(group, path, pathPainter->getCurrentPath().at(i));

    setVisiblePath(path);
    /// resets the menu so that it reflects the creation of this new path
    /// TODO Check that we actually need a parameter -> if it's always currentgroupname then no need
    setPathsGroup(pathCreationWidget->getCurrentGroupName());

    serializePaths(QDir::currentPath() + QDir::separator() + "paths.dat");

    /// TODO CHeck if parameters needed
    updateDisplayedPath(pathCreationWidget->getCurrentGroupName(),
                        pathCreationWidget->getNameEdit()->text().simplified(),
                        getCurrentPathFromPathPainter(),
                        getVisiblePath());

    pathPainter->updatePathPainterSlot(true);
    return already_existed;
}

void PathsController::displayPathSlot(const QString groupName, const QString pathName, const bool display){

    qDebug() << "PathsController::displayPathSlot called on group :" << groupName << ", path :" << pathName << ", display :" << display;
    if(display){
        bool foundFlag = false;
        Paths::Path path = paths->getPath(groupName, pathName, foundFlag);
        if(foundFlag)
            pathPainter->setCurrentPath(path, pathName);
        else
            qDebug() << "MainWindow::displayPathSlot Sorry could not find the path";
    }
    else
         pathPainter->resetPathSlot();
}

void PathsController::displayGroupPaths(){
    const QString current_group = getGroupPathsChecked();

    ///updates the current group displayed
    pathCreationWidget->setCurrentGroupName(current_group);

    /// updates the label to display the name of the group
    pathGroup->getGroupNameLabel()->setText(current_group);
    setPathsGroup(current_group);
    hideGroupsPathsWidget();
    showGroupsPathsWidget();
}

void PathsController::editGroupPaths(){
    qDebug() << "PathsController::editGroupPaths called";

    /// resets the line edit
    groupsPathsWidget->getButtonGroup()->getModifyEdit()->setText("");

    groupsPathsWidget->getActionButtons()->getEditButton()->setToolTip("Choose a new name for your group and press the ENTER key");

    int btnIndex = groupsPathsWidget->getButtonGroup()->getButtonGroup()->checkedId();

    groupsPathsWidget->setCreatingGroup(false);

    /// hides the button so we can show the QLineEdit on top of it
    groupsPathsWidget->getButtonGroup()->getButtonGroup()->button(btnIndex)->hide();

    /// disables the buttons
    groupsPathsWidget->getActionButtons()->enableAll(false);

    /// disables the QButtonGroup
    groupsPathsWidget->getButtonGroup()->setEnabledGroup(false);

    groupsPathsWidget->getButtonGroup()->getModifyEdit()->setPlaceholderText(groupsPathsWidget->getLastCheckedButton());
    groupsPathsWidget->getButtonGroup()->getModifyEdit()->setFocus();
    groupsPathsWidget->getButtonGroup()->getModifyEdit()->show();
    groupsPathsWidget->getButtonGroup()->getModifyEdit()->setFocusPolicy(Qt::FocusPolicy::StrongFocus);

    groupsPathsWidget->getButtonGroup()->getLayout()->removeWidget(groupsPathsWidget->getButtonGroup()->getModifyEdit());
    groupsPathsWidget->getButtonGroup()->getLayout()->insertWidget(btnIndex, groupsPathsWidget->getButtonGroup()->getModifyEdit());
}

void PathsController::prepareGroupPathsCreation(){
    groupsPathsWidget->setCreatingGroup(true);

    /// unchecks the potential checked button
    groupsPathsWidget->getButtonGroup()->uncheck();

    /// resets the name edit field
    groupsPathsWidget->getGroupNameEdit()->setText("");

    /// stops a user from creating a new group with no name
    groupsPathsWidget->getSaveButton()->setEnabled(false);

    /// uncheck and disable the buttons
    groupsPathsWidget->getActionButtons()->checkAll(false);
    groupsPathsWidget->getActionButtons()->enableAll(false);
    groupsPathsWidget->getActionButtons()->getPlusButton()->setToolTip("Enter a name for your group and click \"save\" or click \"cancel\" to cancel");

    /// here we allow a user to create a new group
    groupsPathsWidget->getGroupNameEdit()->show();
    groupsPathsWidget->getGroupNameLabel()->show();
    groupsPathsWidget->getCancelButton()->show();
    groupsPathsWidget->getSaveButton()->show();

    groupsPathsWidget->getGroupNameEdit()->setFocus();
}

/// returns true if the name has really changed
bool PathsController::modifyGroupPathsWithEnter(QString name){
    bool name_has_changed(false);
    name = name.simplified();
    /// enables the plus button
    groupsPathsWidget->getActionButtons()->getPlusButton()->setEnabled(true);

    groupsPathsWidget->getButtonGroup()->getModifyEdit()->hide();
    /// TODO check that the delay caused by this function is not chiant vis a vis de l appel a leftMenu->get... in MainWindow::modifyGroupPathsWithEnter
    if(name.compare(groupsPathsWidget->getLastCheckedButton())){
        name_has_changed = true;
        /// Update the model
        QSharedPointer<Paths::CollectionPaths> value = paths->getGroups()->value(getGroupPathsChecked());
        paths->getGroups()->remove(groupsPathsWidget->getButtonGroup()->getButtonGroup()->checkedButton()->text());
        paths->getGroups()->insert(name, value);
        groupsPathsWidget->updateGroupsPaths();
    } else
        groupsPathsWidget->updateGroupsPaths();

    groupsPathsWidget->setLastCheckedButton("");

    return name_has_changed;
}

void PathsController::doubleClickOnPathsGroup(const QString checkedButton){
    setPathsGroup(checkedButton);
    pathCreationWidget->setCurrentGroupName(checkedButton);
    pathGroup->getGroupNameLabel()->setText(checkedButton);
    hideGroupsPathsWidget();
    pathGroup->show();
}

void PathsController::doubleClickOnPath(const QString pathName, const QString groupName){
    bool foundFlag = false;
    displaySelectedPath->updatePath(groupName, pathName, paths->getPath(groupName, pathName, foundFlag), getVisiblePath());
    hidePathGroupWidget();
    displaySelectedPath->show();
    pathGroup->getPathButtonGroup()->uncheck();
}

void PathsController::checkEyeButtonSlot(const QString path){
    if(!getVisiblePath().compare(path))
        pathGroup->getActionButtons()->getMapButton()->setChecked(true);
}

/// sets the eye icon properly in front of the displayed path if such path exists
void PathsController::updateDisplayedPath(){
    foreach(QAbstractButton* button, pathGroup->getPathButtonGroup()->getButtonGroup()->buttons()){
        if(!button->text().compare(getVisiblePath()))
            button->setIcon(QIcon(":/icons/eye.png"));
        else
            button->setIcon(QIcon(":/icons/blank.png"));
        button->setIconSize(s_icon_size);
    }
}

void PathsController::setPathsGroup(const QString groupName){
    pathGroup->getPathButtonGroup()->deleteButtons();
    /// if the group of paths exists
    if(paths->getGroups()->find(groupName) != paths->getGroups()->end()){
        /// we iterate over it to create the buttons
        QSharedPointer<Paths::CollectionPaths> current_group = paths->getGroups()->value(groupName);
        QMapIterator<QString, QSharedPointer<Paths::Path>> it_paths(*current_group);
        int i(0);
        while(it_paths.hasNext()){
            it_paths.next();
            qDebug() << "found this path" << it_paths.key();
            CustomPushButton* groupButton = new CustomPushButton(it_paths.key(), pathGroup);
            groupButton->setIconSize(s_icon_size);

            /// if this path is displayed on the map we also add an icon to show it on the button
            if(!getVisiblePath().compare(it_paths.key()))
                groupButton->setIcon(QIcon(":/icons/eye.png"));
            else
                groupButton->setIcon(QIcon(":/icons/blank.png"));

            pathGroup->getPathButtonGroup()->getButtonGroup()->addButton(groupButton, i++);
            /// connects the button to the main window to handle double clicks on the button
            connect(groupButton, SIGNAL(doubleClick(QString)), static_cast<MainWindow*> (parent()), SLOT(doubleClickOnPath(QString)));
            groupButton->setCheckable(true);
            pathGroup->getPathButtonGroup()->getLayout()->addWidget(groupButton);
        }
    }
}

void PathsController::displayPath(const QString groupName){
    QString pathName = pathGroup->getLastCheckedButton();
    qDebug() << "MainWindow::displayPath called" << groupName << pathName;
    bool foundFlag = false;
    displaySelectedPath->updatePath(groupName, pathName, getPath(groupName, pathName, foundFlag), pathPainter->getVisiblePath());
    pathGroup->hide();
    displaySelectedPath->show();
    /// resets the widget
    pathGroup->getPathButtonGroup()->uncheck();
    pathGroup->getPathButtonGroup()->setCheckable(true);
}

void PathsController::updatePaths(const Point &old_point, const Point &new_point){
    paths->updatePaths(old_point, new_point);
    /// saves the paths as the paths of the current configuration
    /// for paths to be saved permanently "save map" must be clicked (map menu)
    serializePaths(QDir::currentPath() + QDir::separator() + "paths.dat");
}
