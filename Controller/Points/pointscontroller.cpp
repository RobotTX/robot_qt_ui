#include "pointscontroller.h"
#include <QDir>
#include <QDebug>
#include <QMessageBox>
#include <QAbstractButton>
#include <assert.h>
#include "Helper/helper.h"
#include "Controller/mainwindow.h"
#include "Controller/Map/mapcontroller.h"
#include "Controller/Robots/commandcontroller.h"
#include "Controller/TopLayout/toplayoutcontroller.h"
#include "Controller/Robots/robotscontroller.h"
#include "Model/Other/xmlparser.h"
#include "Model/Paths/pathpoint.h"
#include "Model/Robots/robots.h"
#include "View/Points/pointsleftwidget.h"
#include "View/Points/createpointwidget.h"
#include "View/Points/displayselectedpoint.h"
#include "View/Points/displayselectedgroup.h"
#include "View/Points/displayselectedpointrobots.h"
#include "View/Other/custompushbutton.h"
#include "View/Other/customlabel.h"
#include "View/Points/groupbuttongroup.h"
#include "View/Other/stylesettings.h"
#include "View/LeftMenu/leftmenu.h"

PointsController::PointsController(MainWindow* mainWindow) : QObject(mainWindow){
    points = QSharedPointer<Points>(new Points(this, mainWindow));
    pointsLeftWidget = Q_NULLPTR;
    createPointWidget = Q_NULLPTR;
    displaySelectedPoint = Q_NULLPTR;
    displaySelectedGroup = Q_NULLPTR;
    editedPointView = QSharedPointer<PointView>();

    connect(this, SIGNAL(resetPathPointViews()), mainWindow, SLOT(resetPathPointViewsSlot()));
    connect(this, SIGNAL(setMessageTop(QString,QString)), mainWindow->getTopLayoutController(), SLOT(setLabel(QString,QString)));
    connect(this, SIGNAL(setTemporaryMessageTop(QString,QString,int)), mainWindow->getTopLayoutController(), SLOT(setLabelDelay(QString,QString,int)));
    connect(this, SIGNAL(enableTopLayout(bool)), mainWindow->getTopLayoutController(), SLOT(enableLayout(bool)));
    connect(this, SIGNAL(backEvent()), mainWindow, SLOT(backEvent()));
    connect(this, SIGNAL(setSelectedRobot(QPointer<RobotView>)), mainWindow, SLOT(setSelectedRobot(QPointer<RobotView>)));
    connect(this, SIGNAL(setSelectedTmpPoint()), mainWindow, SLOT(setSelectedTmpPoint()));
}

void PointsController::initializePoints(void){
    qDebug() << "initializing points from" << QDir::currentPath() + QDir::separator() + "points.xml";
    /// retrieves the points from the xml file and stores them in the model
    loadPoints(QDir::currentPath() + QDir::separator() + "points.xml");
    points->addTmpPoint();
}

void PointsController::initializeMenus(MainWindow* mainWindow,
                                       const QSharedPointer<Robots> &robots,
                                       const QSharedPointer<Map> &map){
    /// Menu which display the list of points
    pointsLeftWidget = new PointsLeftWidget(mainWindow, points);
    pointsLeftWidget->hide();
    connect(pointsLeftWidget, SIGNAL(updateGroupButtonGroup()), this, SLOT(updateGroupButtonGroupSlot()));


    /// Menu to edit the selected point
    createPointWidget = new CreatePointWidget(mainWindow);
    /// to display appropriate messages when a user attemps to create a point
    connect(this, SIGNAL(invalidName(QString, PointsController::PointNameError)), mainWindow, SLOT(setMessageCreationPoint(QString, PointsController::PointNameError)));
    createPointWidget->hide();

    /// TODO the 2 next lines of codes are causing a warning about the layout
    /// to display the information relative to a point
    displaySelectedPoint = new DisplaySelectedPoint(mainWindow, robots, points, map);
    displaySelectedPoint->hide();

    /// to display the information relative to a group of points
    displaySelectedGroup = new DisplaySelectedGroup(mainWindow, points);
    displaySelectedGroup->hide();
}

void PointsController::savePoints(const QString fileName){
    XMLParser parser(fileName);
    parser.save(*points);
}

void PointsController::loadPoints(const QString fileName){
    XMLParser parser(fileName);
    parser.readPoints(points);
}

void PointsController::updateGroupDisplayed(const QString groupName){
    displaySelectedGroup->getPointButtonGroup()->setGroup(groupName, points);
}

void PointsController::hidePointViewsToDisplayButPath(QVector<QSharedPointer<PathPoint>> currentPath){
    for(int i = 0; i < pointViewsToDisplay.size(); i++){
        bool hidePointView(true);
        for(int j = 0; j < currentPath.size(); j++){
            if(currentPath.at(j)->getPoint() == *(pointViewsToDisplay.at(i)->getPoint())){
                hidePointView = false;
                break;
            }
        }
        if(hidePointView)
            pointViewsToDisplay.at(i)->hide();
    }
    pointViewsToDisplay.clear();
}

void PointsController::showPointViewsToDisplay(void){
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();
        if(i.value() && i.key().compare(PATH_GROUP_NAME) != 0){
            for(int j = 0; j < i.value()->count(); j++){
                QSharedPointer<PointView> pointView = i.value()->at(j);
                if(pointView && !pointView->isVisible() && pointView->getPoint()->getName().compare(TMP_POINT_NAME)){
                    pointView->show();
                    pointViewsToDisplay.push_back(pointView);
                }
            }
        }
    }
}

void PointsController::hidePointViewsToDisplay(void){
    for(int i = 0; i < pointViewsToDisplay.size(); i++)
        pointViewsToDisplay.at(i)->hide();
    pointViewsToDisplay.clear();
}

void PointsController::editTmpPathPoint(const int id, const int nbWidget){

    editedPointView = points->getGroups()->value(PATH_GROUP_NAME)->at(id);
    editedPointView->setState(GraphicItemState::EDITING_PATH);

    qDebug() << "PointsController::editTmpPathPointSlot number of widget with the same pointView : " << nbWidget;
    /// if 2 path points have the same pointView, we need to create a copy to only
    /// move one of the two path points
    if(nbWidget > 1){
        editedPointView = points->createPoint(
                    editedPointView->getPoint()->getName(),
                    editedPointView->getPoint()->getPosition().getX(),
                    editedPointView->getPoint()->getPosition().getY(),
                    true, Point::PointType::PATH);
        points->getGroups()->value(PATH_GROUP_NAME)->remove(id);
        points->insertPoint(PATH_GROUP_NAME, id, editedPointView);
    }

    editedPointView->setState(GraphicItemState::EDITING_PATH);

    editedPointView->setFlag(QGraphicsItem::ItemIsMovable);
    /// set by hand in order to keep the colors consistent while editing the point and after
    editedPointView->setPixmap(PointView::PixmapType::SELECTED);

}

void PointsController::replacePoint(int id, QString name){
    points->getGroups()->value(PATH_GROUP_NAME)->remove(id);
    points->insertPoint(PATH_GROUP_NAME, id, points->findPointView(name));
}

void PointsController::setSelectedTmpPoint(MainWindow* mainWindow){
    createPointWidget->getGroupBox()->hide();
    createPointWidget->getGroupLabel()->hide();

    QSharedPointer<PointView> displaySelectedPointView = points->getTmpPointView();

    /// sets the pixmaps of the other points
    points->setPixmapAll(PointView::NORMAL);

    displaySelectedPointView->setPixmap(PointView::MID);

    createPointWidget->setSelectedPoint(displaySelectedPointView);
    createPointWidget->show();
    float x = displaySelectedPointView->getPoint()->getPosition().getX();
    float y = displaySelectedPointView->getPoint()->getPosition().getY();

    if(mainWindow->getMapController()->getPixelColor(x, y).red() >= 254){
        emit setMessageTop(TEXT_COLOR_INFO, "To save this point permanently click the \"+\" button");
        createPointWidget->getActionButtons()->getPlusButton()->setEnabled(true);
        createPointWidget->getActionButtons()->getPlusButton()->setToolTip("Click this button if you want to save this point permanently");
    } else {
        emit setMessageTop(TEXT_COLOR_WARNING, "You cannot save this point because your robot(s) would not be able to go there");
        createPointWidget->getActionButtons()->getPlusButton()->setEnabled(false);
        createPointWidget->getActionButtons()->getPlusButton()->setToolTip("You cannot save this point because your robot(s) cannot go there");
    }

    displaySelectedPoint->hide();
}

void PointsController::plusGroupBtnEvent(){
    emit setMessageTop(TEXT_COLOR_INFO, "The name of your group cannot be empty");
    qDebug() << "PointsController::plusGroupBtnEvent called";

    pointsLeftWidget->getGroupNameEdit()->setFocus();
    pointsLeftWidget->setCreatingGroup(true);
    pointsLeftWidget->getGroupButtonGroup()->uncheck();
    /// resets the name edit field
    pointsLeftWidget->getGroupNameEdit()->setText("");
    /// stops a user from creating a new group with no name
    pointsLeftWidget->getSaveButton()->setEnabled(false);
    /// uncheck and disable the buttons
    pointsLeftWidget->getActionButtons()->checkAll(false);

    pointsLeftWidget->getActionButtons()->enableAll(false);

    pointsLeftWidget->getActionButtons()->getPlusButton()->setToolTip("Enter a name for your group and click \"save\" or click \"cancel\" to cancel");

    /// to prevent the user from clicking on the buttons
    pointsLeftWidget->getGroupButtonGroup()->setEnabled(false);

    /// here we allow a user to create a new group

    pointsLeftWidget->getGroupNameLabel()->show();

    pointsLeftWidget->getGroupNameEdit()->show();

    pointsLeftWidget->getCancelButton()->show();
    pointsLeftWidget->getSaveButton()->show();

    pointsLeftWidget->getGroupNameEdit()->setFocus();
}

/**
 * @brief PointsController::minusGroupBtnEvent
 * called in the first points menu to either remove a group or a point which belongs to the default group
 */
void PointsController::minusGroupBtnEvent(){
    qDebug() << "minusGroupBtnEvent called";

    /// uncheck the other buttons
    pointsLeftWidget->getActionButtons()->getPlusButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getEditButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getGoButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getMapButton()->setChecked(false);

    /// we hide those in case the previous button clicked was the plus button
    pointsLeftWidget->getGroupNameEdit()->hide();
    pointsLeftWidget->getGroupNameLabel()->hide();

    QString checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton()->text();

    /// we have to delete a group
    if(points->isAGroup(checkedId))
        askForDeleteGroupConfirmation(checkedId);
    /// we have to delete a point
    else if(points->isAPoint(checkedId))
        askForDeleteDefaultGroupPointConfirmation(checkedId);

    displaySelectedGroup->getPointButtonGroup()->setGroup(displaySelectedGroup->getPointButtonGroup()->getGroupName(), points);
    updateGroupButtonGroupSlot();
}

/**
 * @brief PointsController::askForDeleteGroupConfirmation
 * @param groupName
 * Called when a user wants to remove a whole group of points
 */
void PointsController::askForDeleteGroupConfirmation(QString groupName){
    qDebug() << "askForDeleteGroupConfirmation called";
    QMessageBox msgBox;
    msgBox.setText("Do you really want to remove this group ? All the points in this group will also be removed.");
    msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::Cancel);

    int ret = msgBox.exec();
    switch(ret){
        case QMessageBox::Cancel :
            pointsLeftWidget->getActionButtons()->getMinusButton()->setChecked(false);
            pointsLeftWidget->setLastCheckedId("");
            pointsLeftWidget->disableButtons();
            points->setPixmapAll(PointView::PixmapType::NORMAL);
        break;
        case QMessageBox::Ok : {
            /// we have to check that none of the points is the home of a robot
            QVector<QString> homePointNames = points->getHomeNameFromGroup(groupName);
            if(homePointNames.size() <= 0){

                /// removes all the points of the group on the map
                for(int i = 0; i < points->getGroups()->value(groupName)->size(); i++)
                    points->getGroups()->value(groupName)->at(i)->hide();

                /// removes the group from the model
                points->removeGroup(groupName);

                /// updates the file
                savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

                /// updates the group box so that the user cannot create a point in this group anymore
                createPointWidget->updateGroupBox(points);

            } else {
                /// this group contains the home point of a robot and cannot be removed,
                /// we prompt the end user with a customized message to explain
                /// which robot has its home point in the group
                QString pointsNameStr = "";
                for(int i = 0; i < homePointNames.size(); i++){
                    if(i > 0)
                        pointsNameStr += ", ";
                    pointsNameStr += homePointNames.at(i);
                }
                QMessageBox msgBox;
                msgBox.setText("This group contains the point(s) : " + pointsNameStr
                               + " which are home to their respective robot."
                               + " If you want to remove it you first have to indicate a new home point for this robot.");
                msgBox.setIcon(QMessageBox::Critical);
                msgBox.setStandardButtons(QMessageBox::Ok);
                msgBox.setInformativeText("To modify the home point of a robot you can either click on the menu > Robots, choose a robot and Add Home or simply click a robot on the map and Add Home");
                msgBox.exec();
                qDebug() << "Sorry this point is the home of a robot and therefore cannot be removed";

            }

            /// updates the menu
            pointsLeftWidget->getGroupButtonGroup()->updateButtons(points);
            pointsLeftWidget->getActionButtons()->getMinusButton()->setChecked(false);
            pointsLeftWidget->setLastCheckedId("");
            pointsLeftWidget->disableButtons();
            points->setPixmapAll(PointView::PixmapType::NORMAL);

            if(homePointNames.size() <= 0)
                emit setTemporaryMessageTop(TEXT_COLOR_SUCCESS, "You have successfully deleted the group \"" + groupName + "\"", 2500);
        }
        break;
        default:
            Q_UNREACHABLE();
            /// should never be here
        break;
    }
}

/**
 * @brief PointsController::askForDeleteDefaultGroupPointConfirmation
 * @param pointName
 * Called when a user wants to remove a point that belongs to the default group
 * the pointName given is the name of the point within its group
 */
void PointsController::askForDeleteDefaultGroupPointConfirmation(QString pointName){
    qDebug() << "askForDeleteDefaultGroupPointConfirmation called" << pointName;
    QMessageBox msgBox;
    msgBox.setText("Do you really want to remove this point ?");
    msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::Cancel);

    int ret = msgBox.exec();
    switch(ret){
        case QMessageBox::Cancel :
            pointsLeftWidget->getActionButtons()->getMinusButton()->setChecked(false);
            pointsLeftWidget->setLastCheckedId("");
            pointsLeftWidget->disableButtons();
            points->setPixmapAll(PointView::PixmapType::NORMAL);
        break;
        case QMessageBox::Ok : {
            /// we first check that our point is not the home of a robot
            QSharedPointer<Point> point = points->findPoint(pointName);
            if(!point->isHome()){
                qDebug() << "Go ahead and remove me I am not a home point anyway";

                /// updates the model
                points->removePoint(pointName);

                /// save changes in the file
                savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

                /// updates the menu
                pointsLeftWidget->getGroupButtonGroup()->updateButtons(points);

                /// need to remove the point from the map
                pointsLeftWidget->setLastCheckedId("");

                setMessageTop(TEXT_COLOR_SUCCESS, "You have successfully deleted the point \"" + pointName + "\" that used to belong to the default group");
            } else {
                /// this is in fact the home point of a robot, we prompt a customized message to the end user
                QPointer<RobotView> robot = static_cast<MainWindow*>(parent())->getRobotsController()->getRobots()->findRobotUsingHome(pointName);
                if(robot != NULL){
                    openInterdictionOfPointRemovalMessage(pointName, robot->getRobot()->getName());
                    qDebug() << "Sorry this point is the home of a robot and therefore cannot be removed";
                } else {
                    qDebug() << "setSelectedRobotFromPoint : something unexpected happened";
                }
                points->setPixmapAll(PointView::PixmapType::NORMAL);
            }
        }
        pointsLeftWidget->disableButtons();
        break;
        default:
            Q_UNREACHABLE();
            /// should never be here
            qDebug() << "PointsController::askForDeleteDefaultGroupPointConfirmation should not be here";
        break;
    }
}

/**
 * @brief PointsController::openInterdictionOfPointRemovalMessage
 * @param pointName
 * @param robotName
 * opens a message box to notify a user that the point he is trying to remove cannot
 * be removed because it is the home point of a robot
 */
void PointsController::openInterdictionOfPointRemovalMessage(const QString pointName, const QString robotName){
    QMessageBox msgBox;
    msgBox.setText("The point : " + pointName + " that you are trying to remove is the home point of the robot " + robotName +
                   ". If you want to remove it you first have to indicate a new home point for this robot.");
    msgBox.setIcon(QMessageBox::Critical);
    msgBox.setStandardButtons(QMessageBox::Ok);
    msgBox.setInformativeText("To modify the home point of a robot you can either click on the menu > Robots, choose a robot and Add Home or simply click a robot on the map and Add Home");
    msgBox.exec();
}

/**
 * @brief PointsController::editPointButtonEvent
 * called to edit an existing point
 */
void PointsController::editPointButtonEvent(void){
    emit setMessageTop(TEXT_COLOR_INFO, "Click the map or drag the point to change its position");

    displaySelectedPoint->getNameEdit()->show();
    displaySelectedPoint->getNameLabel()->hide();

    /// update buttons enable attribute and tool tips
    displaySelectedPoint->getActionButtons()->getMapButton()->setChecked(true);
    displaySelectedPoint->getActionButtons()->getMinusButton()->setEnabled(false);
    displaySelectedPoint->getActionButtons()->getMinusButton()->setToolTip("");
    displaySelectedPoint->getActionButtons()->getMapButton()->setEnabled(false);
    displaySelectedPoint->getActionButtons()->getMapButton()->setToolTip("");

    /// hide the temporary point on the map
    QSharedPointer<PointView> displaySelectedPointView = points->findPointView(displaySelectedPoint->getPointName());
    qDebug() << "PointsController::editPointButtonEvent selected point to edit " << displaySelectedPoint->getPointName();
    displaySelectedPointView->setOriginalPosition(displaySelectedPointView->getPoint()->getPosition());

    if(displaySelectedPointView && !(*(displaySelectedPointView->getPoint()) == *(points->getTmpPointView()->getPoint())))
        points->displayTmpPoint(false);

    /// change the color of the pointview that's selected on the map
    displaySelectedPointView->setPixmap(PointView::PixmapType::SELECTED);
    displaySelectedPointView->show();

    pointsLeftWidget->getActionButtons()->getPlusButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getMinusButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getGoButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getMapButton()->setChecked(false);

    /// this way we force the user to either click save or cancel
    displaySelectedPoint->getActionButtons()->getEditButton()->setEnabled(false);
    displaySelectedPoint->getActionButtons()->getEditButton()->setToolTip("You can choose to save or discard your modifications by clicking the save (Enter) and cancel button respectively");

    /// we show the save button and the cancel button
    displaySelectedPoint->getCancelButton()->show();
    displaySelectedPoint->getSaveButton()->show();

    /// sets the state of the map and the other widgets to prevent other concurrent actions
    MainWindow* mainWindow = static_cast<MainWindow*>(parent());
    mainWindow->setGraphicItemsState(GraphicItemState::NO_EVENT);
    mainWindow->getMapController()->setMapState(GraphicItemState::EDITING_PERM);

    /// sets the state of the point of the map to make it draggable
    displaySelectedPointView->setState(GraphicItemState::EDITING_PERM);
    displaySelectedPointView->setFlag(QGraphicsItem::ItemIsMovable, true);

    displaySelectedPoint->getNameEdit()->setText("");
    displaySelectedPoint->getNameEdit()->setPlaceholderText(displaySelectedPointView->getPoint()->getName());
    displaySelectedPoint->getNameEdit()->setFocus();
}

/**
 * @brief PointsController::editGroupBtnEvent
 * called when the user wants to edit a point from the first points menu
 */
void PointsController::editGroupBtnEvent(void){

    if(pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton()){
        qDebug() << "PointsController::editGroupBtnEvent called" << pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton()->text();

        int btnIndex = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedId();

        pointsLeftWidget->setLastCheckedId(static_cast<CustomPushButton*>(pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton())->text());
        assert(pointsLeftWidget->getLastCheckedId().compare(""));
        pointsLeftWidget->setCreatingGroup(false);

        /// uncheck the other buttons
        pointsLeftWidget->getActionButtons()->getPlusButton()->setChecked(false);
        pointsLeftWidget->getActionButtons()->getMinusButton()->setChecked(false);
        pointsLeftWidget->getActionButtons()->getGoButton()->setChecked(false);
        pointsLeftWidget->getActionButtons()->getMapButton()->setChecked(false);

        /// we hide those in case the previous button clicked was the plus button
        pointsLeftWidget->getGroupNameEdit()->hide();
        pointsLeftWidget->getGroupNameLabel()->hide();
        QAbstractButton* btn = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton();
        QString checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton()->text();

        /// it's an isolated point
        if(checkedId.compare("") != 0 && points->isAPoint(checkedId)){

            /// retrieves the pointview associated to the point on the map and displays it if it was not already the case
            QSharedPointer<PointView> pointView = points->findPointView(checkedId);

            MainWindow* mainWindow = static_cast<MainWindow*>(parent());
            /// must display the tick icon in the pointsLeftWidget
            pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton()->setIcon(QIcon(":/icons/eye_point.png"));
            if(pointView){
                pointView->show();
                QString robotName = "";
                if(pointView->getPoint()->isHome()){
                    QPointer<RobotView> robotView = mainWindow->getRobotsController()->getRobots()->findRobotUsingHome(pointView->getPoint()->getName());
                    if(robotView)
                        robotName = robotView->getRobot()->getName();
                    else
                        qDebug() << "editGroupBtnEvent : something unexpected happened";
                }
                displaySelectedPoint->setPointView(pointView, robotName);
            } else {
                qDebug() << "There is no point view associated with those indexes";
            }

            /// displays the information relative the the point
            displaySelectedPoint->displayPointInfo();
            editPointButtonEvent();
            pointsLeftWidget->hide();


            /// disables the back button to prevent problems, a user has to discard or save his modifications before he can start navigatin the menu again, also prevents false manipulations
            displaySelectedPoint->show();
            mainWindow->switchFocus("point", displaySelectedPoint, MainWindow::WidgetType::POINT);
        } else if(checkedId.compare("") != 0 && points->isAGroup(checkedId)){
            qDebug() << "gotta update a group";

            pointsLeftWidget->getActionButtons()->getEditButton()->setToolTip("Type a new name for your group and press ENTER");
            /// disables the plus button
            pointsLeftWidget->getActionButtons()->getPlusButton()->setEnabled(false);
            /// disables the other buttons
            pointsLeftWidget->disableButtons();
            pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->setText("");

            pointsLeftWidget->getGroupButtonGroup()->uncheck();
            pointsLeftWidget->getGroupButtonGroup()->setEnabled(false);

            pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->setText("");
            pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->setPlaceholderText(checkedId);
            pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->setFocus();
            pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->setFocusPolicy(Qt::FocusPolicy::StrongFocus);
            pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->show();
            pointsLeftWidget->getGroupButtonGroup()->getLayout()->removeWidget(pointsLeftWidget->getGroupButtonGroup()->getModifyEdit());
            pointsLeftWidget->getGroupButtonGroup()->getLayout()->insertWidget(btnIndex, pointsLeftWidget->getGroupButtonGroup()->getModifyEdit());

            pointsLeftWidget->getGroupButtonGroup()->setEditedGroupName(checkedId);
            btn->hide();
        }
    }
}

void PointsController::pointSavedEvent(QString groupName, double x, double y, QString name){
    /// resets the status of the plus button
    createPointWidget->getActionButtons()->getPlusButton()->setEnabled(true);

    /// hides widgets relative to the choice of a group
    createPointWidget->hideGroupLayout(true);

    points->addPoint(groupName, name, x, y, true, Point::PointType::TEMP);

    /// saves it to the file
    savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

    /// updates the menu
    updateGroupButtonGroupSlot();

    /// hides the temporary point so that they don't superimpose which is confusing when hiding / showing the newly created point
    points->getTmpPointView()->hide();
}

/**
 * @brief PointsController::removePointFromGroupMenu
 * Called when a user wants to remove a point that belongs to any group but the default one
 * the pointName given is the name of the point within its group
 */
void PointsController::removePointFromGroupMenu(void){
    QString pointName = displaySelectedGroup->getPointButtonGroup()->getButtonGroup()->checkedButton()->text();

    if(pointName.compare("") != 0){
        qDebug() << "PointsController::removePointFromGroupMenu event called" << pointName;
        QMessageBox msgBox;
        msgBox.setText("Do you really want to remove this point ?");
        msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Cancel);

        int ret = msgBox.exec();
        switch(ret){
            case QMessageBox::Cancel :
                qDebug() << "PointsController::removePointFromGroupMenu clicked no";
                displaySelectedGroup->disableButtons();
            break;
            case QMessageBox::Ok :
                {
                    displaySelectedGroup->getPointButtonGroup()->setCheckable(true);
                    /// we first check that our point is not the home of a robot

                    QSharedPointer<PointView> point = points->findPointView(pointName);
                    QString group = points->getGroupNameFromPointName(pointName);
                    if(point && !point->getPoint()->isHome()){
                        qDebug() << "PointsController::removePointFromGroupMenu Go ahead and remove me I am not a home point anyway";
                        /// need to remove the point from the map
                        point->hide();

                        /// updates the model
                        points->removePoint(pointName);

                        /// updates the group menu
                        displaySelectedGroup->getPointButtonGroup()->setGroup(pointsLeftWidget->getLastCheckedId(), points);

                        /// save the changes to the file
                        savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

                        /// makes the buttons checkable again
                        displaySelectedGroup->getPointButtonGroup()->setCheckable(true);

                        /// prompts the user to ask him if he wants to delete the group in case it would be empty
                        if(points->getGroups()->value(group)->size() <= 0){
                            QMessageBox msgBox;
                            msgBox.setText("The group " + group + " is empty. Do you want to delete this group permanently ?");
                            msgBox.setStandardButtons(QMessageBox::No | QMessageBox::Yes);
                            msgBox.setDefaultButton(QMessageBox::No);

                            int res = msgBox.exec();
                            if(res == QMessageBox::Yes){
                                /// updates model
                                points->removeGroup(pointsLeftWidget->getLastCheckedId());

                                /// updates file
                                savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

                                /// updates menu
                                pointsLeftWidget->getGroupButtonGroup()->updateButtons(points);

                                /// hides group menu and shows list of groups menu
                                displaySelectedGroup->hide();
                                pointsLeftWidget->show();
                                createPointWidget->updateGroupBox(points);
                                emit backEvent();
                            }
                        }
                    } else {
                        /// this is in fact the home point of a robot, we prompt a customized message to the end user
                        MainWindow* mainWindow = static_cast<MainWindow*>(parent());
                        QPointer<RobotView> robotView = mainWindow->getRobotsController()->getRobots()->findRobotUsingHome(pointName);
                        if(robotView != NULL){
                            qDebug() << robotView->getRobot()->getName();
                            openInterdictionOfPointRemovalMessage(pointName, robotView->getRobot()->getName());
                            qDebug() << "PointsController::removePointFromGroupMenu Sorry this point is the home of a robot and therefore cannot be removed";
                        } else {
                            qDebug() << "PointsController::removePointFromGroupMenu  : something unexpected happened";
                        }
                    }
                    points->setPixmapAll(PointView::PixmapType::NORMAL);
                    displaySelectedGroup->disableButtons();
                    if(point && !point->getPoint()->isHome())
                        emit setTemporaryMessageTop(TEXT_COLOR_SUCCESS, "You have successfully deleted the point \"" + pointName + "\"", 2500);
                }

            break;
            default:
                Q_UNREACHABLE();
            /// should never be here
                qDebug() << "PointsController::removePointFromGroupMenu  should not be here";
            break;
        }
    } else
        qDebug() << "PointsController::removePointFromGroupMenu can't remove point without name";
}


void PointsController::displayPointEvent(QString name, double x, double y){
    qDebug() << "PointsController::displayPointEvent called" << name;
    QSharedPointer<PointView> pointView = points->findPointView(name);
    if(!pointView)
        pointView = points->findPathPointView(x, y);

    if(pointView){
        qDebug() << "PointsController::displayPointEvent this point is a home" << pointView->getPoint()->isHome();
        MainWindow* mainWindow = static_cast<MainWindow*>(parent());
        if(!(*(pointView->getPoint()) == *(points->getTmpPointView()->getPoint()))){
            /// If the point is not a path or is a path but from a permanent point, we display the menu with informations on the point
            if(!(pointView->getPoint()->isPath() && pointView->getPoint()->getName().contains(PATH_POINT_NAME))){

                points->displayTmpPoint(false);

                displaySelectedPoint->getActionButtons()->getMapButton()->setChecked(true);

                QString robotName = "";
                if(pointView->getPoint()->isHome()){
                    QPointer<RobotView> robotView = mainWindow->getRobotsController()->getRobots()->findRobotUsingHome(pointView->getPoint()->getName());
                    if(robotView)
                        robotName = robotView->getRobot()->getName();
                    else
                        qDebug() << "PointsController::displayPointEvent : something unexpected happened";
                }

                displaySelectedPoint->setPointView(pointView, robotName);

                /// so that the points don't stay blue if we click a new point
                points->setPixmapAll(PointView::PixmapType::NORMAL);

                pointView->setState(GraphicItemState::NO_STATE);

                displaySelectedPoint->displayPointInfo();

                mainWindow->hideAllWidgets();

                mainWindow->getLeftMenu()->show();

                displaySelectedPoint->show();
                mainWindow->resetFocus();
                mainWindow->switchFocus(pointView->getPoint()->getName(), displaySelectedPoint, MainWindow::WidgetType::POINT);

                qDebug() << "PointsController::displayPointEvent  : is this point a path ?" << (pointView->getPoint()->isPath()) << pointView->getPoint()->getType();
                if(pointView->getPoint()->isHome()){
                    QPointer<RobotView> robotView = mainWindow->getRobotsController()->getRobots()->findRobotUsingHome(pointView->getPoint()->getName());
                    if(robotView)
                        robotName = robotView->getRobot()->getName();
                    else
                        qDebug() << "PointsController::displayPointEvent  : something unexpected happened";
                }

                if(pointView->getPoint()->isPath()){
                    /// if it's a path point the edition/suppression is forbidden from here
                    displaySelectedPoint->getActionButtons()->getEditButton()->setEnabled(false);
                    displaySelectedPoint->getActionButtons()->getMinusButton()->setEnabled(false);
                }

            } else {
                /// The point is a path' point from a temporary point so we display the page of the robot in which this pathpoint is used
                QPointer<RobotView> robot = mainWindow->getRobotsController()->getRobots()->findRobotUsingTmpPointInPath(pointView->getPoint());
                if(robot){
                    qDebug() << "PointsController::displayPointEvent  At least, I found the robot" << robot->getRobot()->getName();
                    mainWindow->resetFocus();
                    emit setSelectedRobot(robot);
                }
            }
        } else {
            /// It's the tmpPoint
            emit setSelectedTmpPoint();
        }
    } else {
        qDebug() << "PointsController::displayPointEvent could not found the pointView" << name << x << y;
    }
}

void PointsController::displayGroupMapEvent(void){
    qDebug() << "PointsController::displayGroupMapEvent called";

    if(pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton()){
        /// uncheck the other buttons
        pointsLeftWidget->getActionButtons()->getPlusButton()->setChecked(false);
        pointsLeftWidget->getActionButtons()->getMinusButton()->setChecked(false);
        pointsLeftWidget->getActionButtons()->getEditButton()->setChecked(false);
        pointsLeftWidget->getActionButtons()->getGoButton()->setChecked(false);

        /// we hide those in case the previous button clicked was the plus button
        pointsLeftWidget->getGroupNameEdit()->hide();
        pointsLeftWidget->getGroupNameLabel()->hide();

        QString checkedName = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton()->text();

        int checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedId();
        /// we display groups
        if(points->isAGroup(checkedName)){
            if(points->getGroups()->value(checkedName)){
                /// the group was displayed, we now have to hide it (all its points)
                if(points->isDisplayed(checkedName)){

                    /// updates the tooltip of the map button
                    pointsLeftWidget->getActionButtons()->getMapButton()->setToolTip("Click here to display the selected group on the map");
                    pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/folder_space.png"));

                    for(int i = 0; i < points->getGroups()->value(checkedName)->size(); i++){
                        QSharedPointer<PointView> point = points->getGroups()->value(checkedName)->at(i);
                        point->hide();

                    }
                    /// update the file
                    savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

                } else if(points->getGroups()->value(checkedName)->size() == 0) {
                    pointsLeftWidget->getActionButtons()->getMapButton()->setChecked(false);
                    emit setTemporaryMessageTop(TEXT_COLOR_WARNING, "This group is empty. There is no points to display", 4000);
                } else {
                    /// updates the tooltip of the map button
                    pointsLeftWidget->getActionButtons()->getMapButton()->setToolTip("Click here to hide the selected group on the map");
                    /// the group must now be displayed
                    pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/folder_eye.png"));

                    for(int i = 0; i < points->getGroups()->value(checkedName)->size(); i++){
                        QSharedPointer<PointView> point = points->getGroups()->value(checkedName)->at(i);
                        point->show();
                        point->setPixmap(PointView::PixmapType::SELECTED);
                        /// update the file
                        savePoints(QDir::currentPath() + QDir::separator() + "points.xml");
                    }
                }
            }
        }
        /// we display isolated points
        else if(points->isAPoint(checkedName)){
            QSharedPointer<PointView> point = points->findPointView(checkedName);

            /// if the point is displayed we hide it
            if(point && point->isVisible()){
                /// updates the tooltip of the map button
                pointsLeftWidget->getActionButtons()->getMapButton()->setToolTip("Click here to display the selected point on the map");
                point->hide();

                /// update the file
                savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

                /// we remove the tick icon
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/space_point.png"));
            } else {
                /// updates the tooltip of the map button
                pointsLeftWidget->getActionButtons()->getMapButton()->setToolTip("Click here to hide the selected point on the map");
                point->setPixmap(PointView::PixmapType::SELECTED);
                point->show();

                /// update the file
                savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

                /// we add the tick icon
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/eye_point.png"));
            }
        }
    }
}

void PointsController::displayPointMapEvent(){
    qDebug() << "PointsController::displayPointMapEvent called";
    QSharedPointer<PointView> pointView = points->findPointView(displaySelectedPoint->getPointName());
    QPair<QString, int> pointIndexes = points->findPointIndexes(pointView->getPoint()->getName());
    qDebug() << "Indexes are " << pointIndexes.first << pointIndexes.second;
    int checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonIdByName(pointIndexes.first);

    if(pointView && pointView->getPoint()){
        if(pointView->isVisible()){
            qDebug() << "PointsController::displayPointMapEvent hiding" << pointView->getPoint()->getName();
            displaySelectedPoint->getActionButtons()->getMapButton()->setToolTip("Click to display this point");
            pointView->hide();

            /// update the file
            savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

            /// we update the group menu
            updateGroupDisplayed(pointIndexes.first);

            /// it's a point that belongs to a group
            if(pointIndexes.first.compare(NO_GROUP_NAME) != 0){
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/folder_space.png"));
                qDebug() << pointIndexes.second;
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/space_point.png"));
            } else {
                /// it's an isolated point
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(pointIndexes.second)->setIcon(QIcon(":/icons/space_point.png"));
            }
        } else {
            qDebug() << "PointsController::displayPointMapEvent showing" << pointView->getPoint()->getName();
            pointView->setPixmap(PointView::PixmapType::SELECTED);
            displaySelectedPoint->getActionButtons()->getMapButton()->setToolTip("Click to hide this point");
            pointView->show();

            /// update the file
            savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

            /// we update the groups menu
            /// it's a point that belongs to a group
            if(pointIndexes.first.compare(NO_GROUP_NAME) != 0){
                qDebug() << pointIndexes.second;
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/eye_point.png"));
                /// we check whether or not the entire group is displayed and update the points left widget accordingly by adding a tick Icon or not
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->setIcon(QIcon(":/icons/folder_eye.png"));
            } else {
                /// it's an isolated point
                pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(pointIndexes.second)->setIcon(QIcon(":/icons/eye_point.png"));
            }
        }
    } else {
        qDebug() << "PointsController::displayPointMapEvent the pointView you are trying to show/hide is NULL";
    }
}

/**
 * @brief MainWindow::displayPointsInGroup
 * called when a user clicks the "eye" button in the Points menu
 */
void PointsController::displayPointsInGroup(void){
    qDebug() << "PointsController::displayPointsInGroup called";
    /// uncheck the other buttons

    pointsLeftWidget->getActionButtons()->getPlusButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getMinusButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getEditButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getMapButton()->setChecked(false);

    /// we hide those in case the previous button clicked was the plus button
    pointsLeftWidget->getGroupNameEdit()->hide();
    pointsLeftWidget->getGroupNameLabel()->hide();

    /// retrieves the id of the checked button within the group of buttons
    QString checkedName = pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->checkedButton()->text();
    MainWindow* mainWindow = static_cast<MainWindow*>(parent());

    /// it's a group
    if(points->isAGroup(checkedName)){
       pointsLeftWidget->setLastCheckedId(checkedName);
       pointsLeftWidget->getActionButtons()->getGoButton()->setChecked(false);
       pointsLeftWidget->hide();

       /// before we display the group of points, we make sure that the graphical object is consistent with the model
       updateGroupDisplayed(checkedName);
       displaySelectedGroup->getPointButtonGroup()->setCheckable(true);
       displaySelectedGroup->show();
       displaySelectedGroup->setName(checkedName);

       mainWindow->switchFocus(checkedName, displaySelectedGroup, MainWindow::WidgetType::GROUP);
       setMessageTop(TEXT_COLOR_INFO, "Click the map to add a permanent point");

    } else if(points->isAPoint(checkedName)){
        /// it's an isolated point
        QSharedPointer<PointView> pointView = points->findPointView(checkedName);
        QString robotName = "";

        if(pointView && pointView->getPoint()->isHome()){
            QPointer<RobotView> robotView = mainWindow->getRobotsController()->getRobots()->findRobotUsingHome(checkedName);
            if(robotView)
                robotName = robotView->getRobot()->getName();
            else
                qDebug() << "PointsController::displayPointsInGroup something unexpected happened";
        }

        displaySelectedPoint->setPointView(pointView, robotName);
        displaySelectedPoint->displayPointInfo();
        displaySelectedPoint->show();

        displaySelectedPoint->getActionButtons()->getMapButton()->setChecked((pointView->isVisible() ? true : false));

        mainWindow->switchFocus("Point", displaySelectedPoint, MainWindow::WidgetType::POINT);
        pointsLeftWidget->getActionButtons()->getGoButton()->setChecked(false);
        pointsLeftWidget->hide();
    }
}

/**
 * @brief PointsController::removePointFromInformationMenu
 * called when a user clicks a point on the map and then tries to remove it clicking the "minus" button
 */
void PointsController::removePointFromInformationMenu(void){
    qDebug() << "PointsController::removepointfrominformationmenu event called";
    QMessageBox msgBox;
    msgBox.setText("Do you really want to remove this point ?");
    msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::Cancel);

    int ret = msgBox.exec();
    displaySelectedPoint->getActionButtons()->getMinusButton()->setChecked(false);
    switch(ret){
        case QMessageBox::Cancel :
            qDebug() << "PointsController::removepointfrominformationmenu clicked no";
        break;
        case QMessageBox::Ok : {
            MainWindow* mainWindow = static_cast<MainWindow*>(parent());
            /// first we check that this point is not a home
            QSharedPointer<PointView> pointView = points->findPointView(displaySelectedPoint->getPointName());
            if(pointView && !pointView->getPoint()->isHome()){
                QString pointName = pointView->getPoint()->getName();

                /// holds the index of the group and the index of a particular point in this group within <points>
                QPair<QString, int> pointIndexes = points->findPointIndexes(pointName);

                if(pointIndexes.first.compare("")!= 0){

                    /// it's an isolated point
                    if(points->isAPoint(pointIndexes.first)){
                        /// need to remove the point from the map
                        pointView->hide();

                        /// updates the model
                        points->removePoint(pointName);

                        /// updates the file containing containing points info
                        savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

                        /// updates the list of points
                        pointsLeftWidget->getGroupButtonGroup()->updateButtons(points);
                        emit backEvent();

                    } else {
                        /// need to remove the point from the map
                        pointView->hide();

                        /// updates the model
                        points->removePoint(pointName);

                        /// updates the file containing containing points info
                        savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

                        /// updates the group menu
                        displaySelectedGroup->getPointButtonGroup()->setGroup(pointsLeftWidget->getLastCheckedId(), points);

                        /// closes the window
                        emit backEvent();

                        /// if the group is empty the user is asked whether or not he wants to delete it
                        if(points->findGroup(pointIndexes.first)->isEmpty() && pointIndexes.first.compare(NO_GROUP_NAME) != 0){
                            QMessageBox msgBox;
                            msgBox.setText("The group " + pointIndexes.first + " is empty. Do you want to delete this group permanently ?");
                            msgBox.setStandardButtons(QMessageBox::No | QMessageBox::Yes);
                            msgBox.setDefaultButton(QMessageBox::No);

                            int res = msgBox.exec();
                            /// the group must be deleted
                            if(res == QMessageBox::Yes){
                                /// updates model
                                points->removeGroup(pointIndexes.first);

                                /// updates file
                                savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

                                /// updates menu
                                pointsLeftWidget->getGroupButtonGroup()->updateButtons(points);
                                createPointWidget->updateGroupBox(points);
                                emit backEvent();
                            }
                        }
                        mainWindow->updateAllPaths(*pointView->getPoint(), Point());
                        emit setTemporaryMessageTop(TEXT_COLOR_SUCCESS, "You have deleted the point : " + pointName + " from the group : " + pointIndexes.first, 2500);
                    }
                } else {
                    qDebug() << "PointsController::removepointfrominformationmenu could not find this point";
                }
            } else {
                /// this point is actually the home point of a robot and therefore cannot be removed
                QPointer<RobotView> robot = mainWindow->getRobotsController()->getRobots()->findRobotUsingHome(pointView->getPoint()->getName());
                if(robot != NULL){
                    qDebug() << "PointsController::removepointfrominformationmenu Sorry this point is the home of a robot and therefore cannot be removed";
                    openInterdictionOfPointRemovalMessage(pointView->getPoint()->getName(), robot->getRobot()->getName());
                } else {
                    qDebug() << "PointsController::removepointfrominformationmenu something unexpected happened";
                }
            }
        }
        break;
        default:
            Q_UNREACHABLE();
            /// should never be here
        break;
    }
}

/**
 * @brief PointsController::editPointFromGroupMenu
 */
void PointsController::editPointFromGroupMenu(void){
    qDebug() << "PointsController::editPointFromGroupMenu called";
    emit setMessageTop(TEXT_COLOR_INFO, "Click the map or drag the point to change its position");
    QString groupName = displaySelectedGroup->getNameLabel()->text();

    qDebug() << "PointsController::editPointFromGroupMenu working on group" << groupName << "and id" << displaySelectedGroup->getPointButtonGroup()->getButtonGroup()->checkedId();

    QString pointName = displaySelectedGroup->getPointButtonGroup()->getButtonGroup()->checkedButton()->text();

    if(pointName.compare("") != 0){
        MainWindow* mainWindow = static_cast<MainWindow*>(parent());

        /// update the pointview and show the point on the map with hover color
        QString robotName = "";
        if(points->findPointView(pointName)->getPoint()->isHome()){
            QPointer<RobotView> robotView = mainWindow->getRobotsController()->getRobots()->findRobotUsingHome(pointName);
            if(robotView)
                robotName = robotView->getRobot()->getName();
            else
                qDebug() << "PointsController::editPointFromGroupMenu : something unexpected happened";
        }
        qDebug() << "PointsController::editPointFromGroupMenu name point u trying to edit" << pointName;
        QSharedPointer<PointView> displaySelectedPointView = points->findPointView(pointName);
        displaySelectedPointView->setOriginalPosition(displaySelectedPointView->getPoint()->getPosition());

        if(displaySelectedPointView){
            qDebug() << "PointsController::editPointFromGroupMenu about to put u orange";

            displaySelectedPoint->setPointView(displaySelectedPointView, robotName);

            displaySelectedPointView->setPixmap(PointView::PixmapType::SELECTED);
            displaySelectedPointView->show();

            /// update the file
            savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

            /// sets the state of the map and the other widgets to prevent other concurrent actions
            mainWindow->setGraphicItemsState(GraphicItemState::NO_EVENT);
            mainWindow->getMapController()->setMapState(GraphicItemState::EDITING_PERM);

            /// sets the state of the point of the map to make it draggable
            displaySelectedPointView->setState(GraphicItemState::EDITING_PERM);
            displaySelectedPointView->setFlag(QGraphicsItem::ItemIsMovable, true);

            displaySelectedPoint->getActionButtons()->getMinusButton()->setEnabled(false);
            displaySelectedPoint->getActionButtons()->getMinusButton()->setToolTip("");
            displaySelectedPoint->getActionButtons()->getMapButton()->setEnabled(false);
            displaySelectedPoint->getActionButtons()->getMapButton()->setToolTip("");
            displaySelectedPoint->getActionButtons()->getMapButton()->setChecked(true);

            /// to force the user to click either the save or the cancel button
            displaySelectedPoint->getActionButtons()->getEditButton()->setEnabled(false);
            displaySelectedPoint->getActionButtons()->getEditButton()->setToolTip("You can choose to save or discard your modifications by clicking the save (Enter) and cancel button respectively");

            displaySelectedPoint->displayPointInfo();

            displaySelectedPoint->getActionButtons()->getEditButton()->setChecked(true);
            displaySelectedPoint->getNameEdit()->setText("");
            displaySelectedPoint->getNameEdit()->setPlaceholderText(pointName);
            displaySelectedPoint->getCancelButton()->show();
            displaySelectedPoint->getSaveButton()->show();
            displaySelectedPoint->getNameEdit()->show();
            displaySelectedPoint->getNameEdit()->setFocus();
            displaySelectedPoint->getNameLabel()->hide();
            displaySelectedPoint->show();
            displaySelectedGroup->hide();
            mainWindow->switchFocus(displaySelectedPoint->getPointName(),displaySelectedPoint, MainWindow::WidgetType::POINT);
        }
    }
}

/**
 * @brief PointsController::displayPointInfoFromGroupMenu
 * display the information of a point from the group menu
 */
void PointsController::displayPointInfoFromGroupMenu(void){
    qDebug() << "PointsController::displayPointInfoFromGroupMen display point info from group menu event called";
    /// retrieves a pointer to the pointView using the text of the label

    QString pointName = displaySelectedGroup->getPointButtonGroup()->getButtonGroup()->checkedButton()->text();
    QSharedPointer<PointView> pointView = points->findPointView(pointName);

    if(pointName.compare("") != 0 && pointView){
        MainWindow* mainWindow = static_cast<MainWindow*>(parent());
        emit setMessageTop(TEXT_COLOR_NORMAL, "");

        QString robotName = "";
        if(pointView->getPoint()->isHome()){
            QPointer<RobotView> robotView = mainWindow->getRobotsController()->getRobots()->findRobotUsingHome(pointName);
            if(robotView)
                robotName = robotView->getRobot()->getName();
            else
                qDebug() << "setSelectedRobotFromPoint : something unexpected happened";
        }

        displaySelectedPoint->setPointView(pointView, robotName);
        displaySelectedPoint->displayPointInfo();

        /// map is checked if the point is displayed
        displaySelectedPoint->getActionButtons()->getMapButton()->setChecked((pointView->isVisible()) ? true : false);
        displaySelectedPoint->show();
        displaySelectedGroup->hide();
        mainWindow->switchFocus(displaySelectedPoint->getPointName(), displaySelectedPoint, MainWindow::WidgetType::POINT);
    } else {
        qDebug() << "PointsController::displayPointInfoFromGroupMen no point named :" << pointName;
    }
}

/**
 * @brief PointsController::updatePoint
 * called when a user edits a point and save the changes either by pressing the enter key or clicking the save button
 */
void PointsController::updatePoint(void){

    qDebug() << "PointsController::updatePoint";
    MainWindow* mainWindow = static_cast<MainWindow*>(parent());
    QSharedPointer<PointView> displaySelectedPointView = points->findPointView(displaySelectedPoint->getPointName());
    /// to update the paths
    Point copy = *displaySelectedPointView->getPoint();
    /// the position of the point before edition, it is needed to give the paths the old point (name and pos) and the new point
    copy.setPosition(displaySelectedPointView->getOriginalPosition().getX(),
                     displaySelectedPointView->getOriginalPosition().getY());

    /// if it is a valid point on the map
    if(mainWindow->getMapController()->getPixelColor(displaySelectedPointView->getPoint()->getPosition().getX(),
                                     displaySelectedPointView->getPoint()->getPosition().getY()).red() >= 254){

        ///resets the tooltip of the edit button and the minus button
        displaySelectedPoint->getActionButtons()->getEditButton()->setToolTip("Click here and then choose between clicking on the map or drag the point to change its position");
        displaySelectedPoint->getActionButtons()->getMinusButton()->setToolTip("Click here to remove the point");

        displaySelectedPoint->getActionButtons()->getMapButton()->setEnabled(true);
        displaySelectedPoint->getActionButtons()->getMapButton()->setToolTip("Click to hide this point");

        /// resets the color of the pointView
        points->getTmpPointView()->setPixmap(PointView::PixmapType::NORMAL);

        /// if the field has been left empty we keep the old name
        if(displaySelectedPoint->getNameEdit()->text().simplified().compare("")){
            displaySelectedPoint->getNameLabel()->setText(displaySelectedPoint->getNameEdit()->text().simplified());
        }

        /// updates the name in the label
        if(displaySelectedPoint->getNameEdit()->text().simplified().compare(""))
            displaySelectedPointView->getPoint()->setName(displaySelectedPoint->getNameEdit()->text());

        /// updates the position of the point
        /// to determine wheter the coordinate is 2 digits long or 3 digits long in order to parse them correctly
        int xLength = displaySelectedPoint->getXLabel()->text().count();
        int yLength = displaySelectedPoint->getYLabel()->text().count();

        displaySelectedPointView->setPos(
        displaySelectedPoint->getXLabel()->text().right(xLength-4).toFloat(),
        displaySelectedPoint->getYLabel()->text().right(yLength-4).toFloat());


        /// save changes to the file
        savePoints(QDir::currentPath() + QDir::separator() + "points.xml");
/**
  TODO COULD CHECK IF ROBOT IS CONNECTED

  IF YES SEND -> IF NOT RECEIVED -> CANCEL

  IF NO -> UPDATE -> ROBOT WILL RECEIVE UPON CONNECTION

  **/
        if(displaySelectedPointView->getPoint()->isHome()){
            /// if the point is the home of a robot, we update the file containing the home on the robot
            qDebug() << "MainWindow::updatePoint need to update if home";
            Position posInRobotCoordinates = Helper::Convert::pixelCoordToRobotCoord(
                        displaySelectedPointView->getPoint()->getPosition(),
                        mainWindow->getMapController()->getMapOrigin().getX(),
                        mainWindow->getMapController()->getMapOrigin().getY(),
                        mainWindow->getMapController()->getMapResolution(),
                        mainWindow->getMapController()->getMapHeight());
            QPointer<RobotView> robotView = mainWindow->getRobotsController()->getRobots()->getRobotViewByName(displaySelectedPointView->getPoint()->getRobotName());
            if(robotView)
                mainWindow->getCommandController()->sendCommand(robotView->getRobot(), QString("n \"") + QString::number(posInRobotCoordinates.getX()) + "\" \""
                                                                    + QString::number(posInRobotCoordinates.getY()) + "\"", displaySelectedPoint->getPointName(), "", "", false, 1);
            else {
                cancelUpdatePoint();
                emit setMessageTop(TEXT_COLOR_DANGER, displaySelectedPointView->getPoint()->getRobotName() + " did not receive the updated coordinates of its new home. You may want to use the \"Assign a home\" button"
                                                                                                        "to send your robot its home again.");
            }
        }

        /// so that you cannot edit a new name unless you click the edit button again
        displaySelectedPoint->getActionButtons()->getEditButton()->setChecked(false);

        /// we hide the save button and the cancel button
        displaySelectedPoint->getCancelButton()->hide();
        displaySelectedPoint->getSaveButton()->hide();

        /// reset the state of the map so we can click it again
        mainWindow->setGraphicItemsState(GraphicItemState::NO_STATE);

        /// enable the edit button and the minus button again
        displaySelectedPoint->getActionButtons()->getEditButton()->setEnabled(true);
        displaySelectedPoint->getActionButtons()->getMinusButton()->setEnabled(true);

        /// updates the isolated points in the group menus
        pointsLeftWidget->getGroupButtonGroup()->updateButtons(points);

        /// we enable the "back" button again
        mainWindow->getLeftMenu()->getReturnButton()->setEnabled(true);
        mainWindow->getLeftMenu()->getReturnButton()->setToolTip("");

        /// We update the paths as this point might have been used in a path
        mainWindow->updateAllPaths(copy, *displaySelectedPointView->getPoint());
        qDebug() << "copy" << copy.getName() << copy.getPosition().getX() << copy.getPosition().getY();
        qDebug() << " new" << displaySelectedPointView->getPoint()->getName() << displaySelectedPointView->getPoint()->getPosition().getX() << displaySelectedPointView->getPoint()->getPosition().getY();

        displaySelectedPoint->getNameEdit()->hide();
        displaySelectedPoint->getNameLabel()->show();

        emit setTemporaryMessageTop(TEXT_COLOR_SUCCESS, "Your point has been successfully updated", 4000);
    } else {
        emit setTemporaryMessageTop(TEXT_COLOR_DANGER, "You cannot save your point \"" +
                               displaySelectedPointView->getPoint()->getName() +
                               "\" because this area of the map is not known to your robot(s), as a result your robot(s) would not be able to move there", 3500);
        /// otherwise the point is not saved and an error message is displayed at the top
    }
}


/**
 * @brief PointsController::cancelUpdatePoint
 * called when a user discard the changes made about a point
 */
void PointsController::cancelUpdatePoint(void){
    qDebug() << "PointsController::cancelUpdatePoint called";
    MainWindow* mainWindow = static_cast<MainWindow*>(parent());
    displaySelectedPoint->getNameEdit()->hide();
    displaySelectedPoint->getNameLabel()->show();
    mainWindow->getLeftMenu()->getCloseButton()->setEnabled(true);
    mainWindow->getLeftMenu()->getReturnButton()->setEnabled(true);
    emit enableTopLayout(true);
    /// reset the color of the pointView
    QSharedPointer<PointView> displaySelectedPointView = points->findPointView(displaySelectedPoint->getPointName());
    if(displaySelectedPointView){
        displaySelectedPointView->setPixmap(PointView::PixmapType::NORMAL);
        points->getTmpPointView()->setPixmap(PointView::PixmapType::NORMAL);
        qDebug() << "PointsController::cancelUpdatePoint about to reset your position";

        displaySelectedPointView->getPoint()->setPosition(displaySelectedPointView->getOriginalPosition());

        /// we hide the buttons relative to the edit option and make sure the points properties are not longer modifiable
        displaySelectedPoint->getActionButtons()->getEditButton()->setChecked(false);
        displaySelectedPoint->getCancelButton()->hide();
        displaySelectedPoint->getSaveButton()->hide();

        /// in case the user had dragged the point around the map or clicked it, this resets the coordinates displayed to the original ones
        displaySelectedPoint->getXLabel()->setText(QString("X : ") + QString::number(
                                                                      displaySelectedPointView->getPoint()->getPosition().getX()));
        displaySelectedPoint->getYLabel()->setText(QString("Y : ") + QString::number(
                                                                      displaySelectedPointView->getPoint()->getPosition().getY()));
        /// enable the edit button again and the map button
        displaySelectedPoint->getActionButtons()->getEditButton()->setEnabled(true);
        displaySelectedPoint->getActionButtons()->getEditButton()->setToolTip("You can click on this button and then choose between clicking on the map or drag the point to change its position");
        displaySelectedPoint->getActionButtons()->getMapButton()->setEnabled(true);
        displaySelectedPoint->getActionButtons()->getMapButton()->setToolTip("Click to hide this point");

        /// resets the tooltip of the minus button
        displaySelectedPoint->getActionButtons()->getMinusButton()->setToolTip("You can click this button to remove the point");
        displaySelectedPoint->getActionButtons()->getMinusButton()->setEnabled(true);

        /// reset the state of the map so we can click it again
        mainWindow->setGraphicItemsState(GraphicItemState::NO_STATE);
        displaySelectedPointView->setPos(static_cast<qreal>(displaySelectedPointView->getPoint()->getPosition().getX()),
                                                                    static_cast<qreal>(displaySelectedPointView->getPoint()->getPosition().getY()));
        /// reset its name in the hover on the map
        displaySelectedPoint->getNameEdit()->setText(displaySelectedPointView->getPoint()->getName());
    } else {
        qDebug() << "PointsController::cancelUpdatePoint can't find the pointView :" << displaySelectedPoint->getPointName();
    }
    /// enable the back button in case we were editing coming from the left menu
    mainWindow->getLeftMenu()->getReturnButton()->setEnabled(true);
    mainWindow->getLeftMenu()->getReturnButton()->setToolTip("");

    emit setMessageTop(TEXT_COLOR_INFO, "Your point \"" + displaySelectedPoint->getPointName() + "\" has not been modified");
}


/**
 * @brief PointsController::updateCoordinates
 * @param x
 * @param y
 * updates the coordinates of the selected point
 */
void PointsController::updateCoordinates(double x, double y){
    qDebug() << "PointsController::updateCoordinates called";
    displaySelectedPoint->getXLabel()->setText("X : " + QString::number(x, 'f', 1));
    displaySelectedPoint->getYLabel()->setText("Y : " + QString::number(y, 'f', 1));
    points->findPointView(displaySelectedPoint->getPointName())->setPos(x, y);

    if(static_cast<MainWindow*>(parent())->getMapController()->getPixelColor(x ,y).red() >= 254){
        displaySelectedPoint->getSaveButton()->setEnabled(true);
        emit setMessageTop(TEXT_COLOR_INFO, "To save this point permanently click \"Save\" or press Enter");
    }
    else {
        displaySelectedPoint->getSaveButton()->setEnabled(false);
        emit setMessageTop(TEXT_COLOR_WARNING, "You cannot save this point because the current position is known as an obstacle for the robot");
    }
}

/**
 * @brief PointsController::displayPointFromGroupMenu
 * called when a user displays or hides a point on the map from the group menu
 */
void PointsController::displayPointFromGroupMenu(){

    const QString pointName = displaySelectedGroup->getPointButtonGroup()->getButtonGroup()->checkedButton()->text();

    qDebug() << "PointsController::displayPointFromGroupMenu event called" << pointName ;

    int checkedId = displaySelectedGroup->getPointButtonGroup()->getButtonIdByName(pointName);

    if(checkedId != -1){
        QSharedPointer<PointView> currentPointView = points->findPointView(pointName);

        /// if the point is displayed we stop displaying it
        if(currentPointView->isVisible()){

            /// hides the point on the map
            currentPointView->hide();

            /// removes the tick icon to show that the point is not displayed on the map
            displaySelectedGroup->getPointButtonGroup()->getButtonGroup()->buttons()[checkedId]->setIcon(QIcon(":/icons/space_point.png"));

            /// updates the file
            savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

            /// if the entire group was displayed it is not the case anymore
            if(pointsLeftWidget->getGroupButtonGroup()->getButtonByName(pointsLeftWidget->getLastCheckedId()) != NULL)
                pointsLeftWidget->getGroupButtonGroup()->getButtonByName(pointsLeftWidget->getLastCheckedId())->setIcon(QIcon(":/icons/folder_space.png"));

            /// changes the map button message
            displaySelectedGroup->getActionButtons()->getMapButton()->setToolTip("Click to display the selected point on the map");

        } else {

            /// shows the point on the map
            currentPointView->show();

            /// we add a tick icon next to the name of the point to show that it is displayed on the map
            displaySelectedGroup->getPointButtonGroup()->getButtonGroup()->buttons()[checkedId]->setIcon(QIcon(":/icons/eye_point.png"));

            /// saves changes to the file
            savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

            /// we check whether or not the entire group is displayed and update the points left widget accordingly by adding a tick Icon or not
            if(points->isDisplayed(pointsLeftWidget->getLastCheckedId())){
                if(pointsLeftWidget->getGroupButtonGroup()->getButtonByName(pointsLeftWidget->getLastCheckedId()) != NULL)
                    pointsLeftWidget->getGroupButtonGroup()->getButtonByName(pointsLeftWidget->getLastCheckedId())->setIcon(QIcon(":/icons/folder_eye.png"));
            }

            /// changes the map button message
            displaySelectedGroup->getActionButtons()->getMapButton()->setToolTip("Click to hide the selected point on the map");

        }
    } else {
        /// should never be here
        Q_UNREACHABLE();
    }
}

/**
 * @brief PointsController::doubleClickOnPoint
 * @param pointName
 * does the same as clicking on a point and then on the eye button
 */
void PointsController::doubleClickOnPoint(QString pointName){
    qDebug() << "PointsController::doubleClickOnPoint called";
    emit setMessageTop(TEXT_COLOR_NORMAL, "");
    QSharedPointer<PointView> pointView = points->findPointView(pointName);

    if(pointView){
        MainWindow* mainWindow = static_cast<MainWindow*>(parent());
        QString robotName = "";
        if(pointView->getPoint()->isHome()){
            QPointer<RobotView> robotView = mainWindow->getRobotsController()->getRobots()->findRobotUsingHome(pointName);
            if(robotView)
                robotName = robotView->getRobot()->getName();
            else
                qDebug() << "PointsController::doubleClickOnPoint something unexpected happened";
        }
        displaySelectedPoint->setPointView(pointView, robotName);
        displaySelectedPoint->displayPointInfo();

        if(pointView->isVisible())
            displaySelectedPoint->getActionButtons()->getMapButton()->setChecked(true);
        else
            displaySelectedPoint->getActionButtons()->getMapButton()->setChecked(false);
        displaySelectedPoint->show();
        displaySelectedGroup->hide();
        mainWindow->switchFocus(displaySelectedPoint->getPointName(), displaySelectedPoint, MainWindow::WidgetType::POINT);
    } else {
        qDebug()  << "PointsController::doubleClickOnPoint no group " << displaySelectedGroup->getNameLabel()->text();
    }
}

/**
 * @brief PointsController::doubleClickOnGroup
 * @param checkedId
 * does the same as clicking on a group or a point belonging to the default group
 * and then on the eye button
 */
void PointsController::doubleClickOnGroup(QString checkedName){
    qDebug() << "PointsController::doubleClickOnGroup called" << checkedName;

    MainWindow* mainWindow = static_cast<MainWindow*>(parent());

    /// uncheck the other buttons
    pointsLeftWidget->getActionButtons()->getPlusButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getMinusButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getEditButton()->setChecked(false);
    pointsLeftWidget->getActionButtons()->getMapButton()->setChecked(false);

    /// we hide those in case the previous button clicked was the plus button
    pointsLeftWidget->getGroupNameEdit()->hide();
    pointsLeftWidget->getGroupNameLabel()->hide();

    /// it's a group
    if(points->isAGroup(checkedName)){
       emit setMessageTop(TEXT_COLOR_INFO, "Click the map to add a permanent point");
       pointsLeftWidget->setLastCheckedId(checkedName);
       pointsLeftWidget->getActionButtons()->getGoButton()->setChecked(false);
       pointsLeftWidget->hide();

       /// before we display the group of points, we make sure that the graphical object is consistent with the model
       updateGroupDisplayed(checkedName);
       displaySelectedGroup->getPointButtonGroup()->setCheckable(true);
       displaySelectedGroup->show();
       displaySelectedGroup->setName(checkedName);

       mainWindow->switchFocus(checkedName, displaySelectedGroup, MainWindow::WidgetType::GROUP);
    } else if(points->isAPoint(checkedName)){

        /// it's an isolated point
        emit setMessageTop(TEXT_COLOR_NORMAL, "");
        QSharedPointer<PointView> pointView = points->findPointView(checkedName);

        QString robotName = "";
        if(pointView->getPoint()->isHome()){
            QPointer<RobotView> robotView = mainWindow->getRobotsController()->getRobots()->findRobotUsingHome(pointView->getPoint()->getName());
            if(robotView)
                robotName = robotView->getRobot()->getName();
            else
                qDebug() << "PointsController::doubleClickOnGroup something unexpected happened";
        }

        displaySelectedPoint->setPointView(pointView, robotName);
        displaySelectedPoint->displayPointInfo();
        displaySelectedPoint->show();

        pointsLeftWidget->getActionButtons()->getGoButton()->setChecked(false);
        if(pointView->isVisible())
            displaySelectedPoint->getActionButtons()->getMapButton()->setChecked(true);
        else
            displaySelectedPoint->getActionButtons()->getMapButton()->setChecked(false);

        pointsLeftWidget->hide();
        mainWindow->switchFocus(checkedName, displaySelectedPoint, MainWindow::WidgetType::POINT);
    }
}


void PointsController::createGroup(QString groupName){
    qDebug() << "PointsController::createGroup called" << groupName;
    MainWindow* mainWindow = static_cast<MainWindow*>(parent());

    groupName = groupName.simplified();
    if(checkGroupName(groupName) == 0){
        pointsLeftWidget->setLastCheckedId("");

        /// updates the model
        points->addGroup(groupName);

        /// updates the file
        savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

        /// updates list of groups in menu
        updateGroupButtonGroupSlot();

        /// updates the comboBox to make this new group available when a user creates a point
        createPointWidget->updateGroupBox(points);

        /// enables the return button again
        mainWindow->getLeftMenu()->getReturnButton()->setEnabled(true);

        /// hides everything that's related to the creation of a group
        pointsLeftWidget->getCancelButton()->hide();
        pointsLeftWidget->getSaveButton()->hide();
        pointsLeftWidget->getGroupNameEdit()->hide();
        pointsLeftWidget->getGroupNameLabel()->hide();

        /// enables the plus button again
        pointsLeftWidget->disableButtons();
        pointsLeftWidget->getActionButtons()->getPlusButton()->setEnabled(true);
        pointsLeftWidget->getActionButtons()->getPlusButton()->setToolTip("Click here to add a new group");
        emit enableTopLayout(true);

        emit setTemporaryMessageTop(TEXT_COLOR_SUCCESS, "You have successfully created a new group : \"" + groupName + "\"", 4000);
    } else if(checkGroupName(groupName) == 1){
        pointsLeftWidget->setLastCheckedId("");

        /// updates list of groups in menu
        updateGroupButtonGroupSlot();

        /// enables the return button again
        mainWindow->getLeftMenu()->getReturnButton()->setEnabled(true);

        /// hides everything that's related to the creation of a group
        pointsLeftWidget->getCancelButton()->hide();
        pointsLeftWidget->getSaveButton()->hide();
        pointsLeftWidget->getGroupNameEdit()->hide();
        pointsLeftWidget->getGroupNameLabel()->hide();

        /// enables the plus button again
        pointsLeftWidget->disableButtons();
        pointsLeftWidget->getActionButtons()->getPlusButton()->setEnabled(true);
        pointsLeftWidget->getActionButtons()->getPlusButton()->setToolTip("Click here to add a new group");
        emit enableTopLayout(true);
    } else
        emit setTemporaryMessageTop(TEXT_COLOR_DANGER, "You cannot choose : " + groupName + " as a new name for your group because another group already has this name", 4000);
}

void PointsController::modifyGroupWithEnter(QString name){
    name = name.simplified();
    qDebug() << "PointsController::modifyGroupWithEnter called : modifying group after enter key pressed from" << pointsLeftWidget->getLastCheckedId() << "to" << name;

    MainWindow* mainWindow = static_cast<MainWindow*>(parent());
    emit enableTopLayout(true);
    mainWindow->setEnableAll(true);

    QString oldGroupName = pointsLeftWidget->getLastCheckedId();
    qDebug() << "PointsController::modifyGroupWithEnter checkgroupname result is" << checkGroupName(name);
    if(checkGroupName(name) == 0){
        qDebug() << "PointsController::modifyGroupWithEnter this name is ok";

        /// Updates the model
        points->getGroups()->insert(name, points->getGroups()->take(oldGroupName));

        /// updates the group box to create a point
        createPointWidget->updateGroupBox(points);

        /// enables the plus button
        pointsLeftWidget->getActionButtons()->getPlusButton()->setEnabled(true);

        /// saves to file
        savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

        /// enables the buttons
        pointsLeftWidget->getGroupButtonGroup()->setEnabled(true);
        pointsLeftWidget->disableButtons();

        /// updates view
        int checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonIdByName(pointsLeftWidget->getGroupButtonGroup()->getEditedGroupName());

        pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->show();
        static_cast<CustomPushButton*> (pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId))->setText(name);
        pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->hide();

        pointsLeftWidget->setLastCheckedId("");

        /// resets the pixmaps
        points->setPixmapAll(PointView::PixmapType::NORMAL);

        emit setTemporaryMessageTop(TEXT_COLOR_SUCCESS, "You have successfully updated the name of your group from \"" + oldGroupName + "\" to \"" + name + "\"", 4000);

    } else if(checkGroupName(name) == 1){
        /// enables the buttons
        pointsLeftWidget->getGroupButtonGroup()->setEnabled(true);
        pointsLeftWidget->disableButtons();

        /// updates view
        int checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonIdByName(pointsLeftWidget->getGroupButtonGroup()->getEditedGroupName());

        pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->show();
        pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->hide();

        /// enables the plus button
        pointsLeftWidget->getActionButtons()->getPlusButton()->setEnabled(true);

        /// resets the pixmaps
        points->setPixmapAll(PointView::PixmapType::NORMAL);

        pointsLeftWidget->setLastCheckedId("");
    }
    else
        emit setTemporaryMessageTop(TEXT_COLOR_DANGER, "You cannot choose : " + name + " as a new name for your group because another group already has this name", 4000);
}

void PointsController::modifyGroupAfterClick(QString name){
    name = name.simplified();
    qDebug() << "PointsController::modifyGroupAfterClick called from" << pointsLeftWidget->getLastCheckedId() << "to" << name;

    MainWindow* mainWindow = static_cast<MainWindow*>(parent());
    emit enableTopLayout(true);

    if (pointsLeftWidget->getLastCheckedId() != "") {
        /// resets the menu
        pointsLeftWidget->getActionButtons()->getPlusButton()->setEnabled(true);
        pointsLeftWidget->getGroupButtonGroup()->setEnabled(true);
        pointsLeftWidget->disableButtons();
        pointsLeftWidget->getGroupButtonGroup()->getModifyEdit()->hide();

        int checkedId = pointsLeftWidget->getGroupButtonGroup()->getButtonIdByName(pointsLeftWidget->getGroupButtonGroup()->getEditedGroupName());
        QString color ="";
        QString msg = "";
        if(checkGroupName(name) == 0){
            /// Update the model
            qDebug() << pointsLeftWidget->getLastCheckedId();
            points->getGroups()->insert(name, points->getGroups()->take(pointsLeftWidget->getLastCheckedId()));

            /// saves to file
            savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

            color = TEXT_COLOR_SUCCESS;
            msg = "You have successfully modified the name of your group";
        } else if(checkGroupName(name) == 1){
            color = TEXT_COLOR_DANGER;
            msg = "The name of your group cannot be empty. Please choose a name for your group";
        } else {
            color = TEXT_COLOR_DANGER;
            msg = "You cannot choose : " + name.simplified() + " as a new name for your group because another group already has this name";
        }

        pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->button(checkedId)->show();
        pointsLeftWidget->setLastCheckedId("");
        emit setTemporaryMessageTop(color, msg, 4000);
    }
}

/**
 * @brief PointsController::reestablishConnections
 * to reestablish the double clicks after points are updated (because buttons in the menu are recreated)
 */
void PointsController::reestablishConnectionsGroups(){
    qDebug() << "PointsController::reestablishConnectionsGroups called";
    foreach(QAbstractButton* button, pointsLeftWidget->getGroupButtonGroup()->getButtonGroup()->buttons())
        connect(button, SIGNAL(doubleClick(QString)), this, SLOT(doubleClickOnGroup(QString)));
}

/**
 * @brief PointsController::reestablishConnections
 * to reestablish the double clicks after groups are updated
 */
void PointsController::reestablishConnectionsPoints(){
    qDebug() << "PointsController::reestablishConnectionsPoints called";
    foreach(QAbstractButton* button, displaySelectedGroup->getPointButtonGroup()->getButtonGroup()->buttons())
        connect(button, SIGNAL(doubleClick(QString)), this, SLOT(doubleClickOnPoint(QString)));
}


void PointsController::showHomeFromRobotName(QString robotName){
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();
        if(i.value() && i.key().compare(PATH_GROUP_NAME) != 0 && i.key().compare(TMP_GROUP_NAME)){
            for(int j = 0; j < i.value()->count(); j++){
                QSharedPointer<PointView> pointView = i.value()->at(j);
                if(pointView->getPoint()->isHome()){
                    if(!pointView->getPoint()->getRobotName().compare(robotName)){
                        /// to make the old home a normal point again
                        pointView->getPoint()->setHome(Point::HOME);
                        pointView->setPixmap(PointView::PixmapType::NORMAL);
                    } else {
                        /// to make it look like a normal point we alter its type temporarily
                        pointView->getPoint()->setHome(Point::PERM);
                        pointView->setPixmap(PointView::PixmapType::NORMAL);
                        pointView->getPoint()->setHome(Point::HOME);
                    }
                }
            }
        }
    }
}

void PointsController::showHomeFromHomeName(QString homeName){
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();
        if(i.value() && i.key().compare(PATH_GROUP_NAME) != 0 && i.key().compare(TMP_GROUP_NAME)){
            for(int j = 0; j < i.value()->count(); j++){
                QSharedPointer<PointView> pointView = i.value()->at(j);
                if(pointView->getPoint()->isHome()){
                    if(pointView->getPoint()->getName().compare(homeName)){
                        /// to make it look like a normal point we alter its type temporarily
                        pointView->getPoint()->setHome(Point::PERM);
                        pointView->setPixmap(PointView::PixmapType::NORMAL);
                        pointView->getPoint()->setHome(Point::HOME);
                    }
                }
            }
        }
    }
}

void PointsController::resetPointViewsSlot(void){
    points->setPixmapAll(PointView::PixmapType::NORMAL);
}

void PointsController::checkPointName(QString name){
    name = Helper::formatName(name);

    createPointWidget->getNameEdit()->setText(name);
    if(name.simplified().contains(QRegularExpression("[;{}]")) || name.contains("pathpoint", Qt::CaseInsensitive)){
        createPointWidget->getSaveBtn()->setToolTip("The name of your point cannot contain the characters \";\" and } or the pattern <pathpoint> ");
        createPointWidget->getSaveBtn()->setEnabled(false);
        emit invalidName(TEXT_COLOR_WARNING, PointNameError::ContainsSemicolon);
        return;
    }
    if(!name.simplified().compare("")){
        qDebug() << " I am empty ";
        /// cannot add a point with no name
        createPointWidget->getSaveBtn()->setToolTip("The name of your point cannot be empty");
        createPointWidget->getSaveBtn()->setEnabled(false);
        emit invalidName(TEXT_COLOR_WARNING, PointNameError::EmptyName);
        return;
    }

    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        /// determines whether or not the current name is a valid one (not already the name of another point or group)
        bool valid(true);
        i.next();
        if(!i.key().compare(name.simplified(), Qt::CaseInsensitive)){
            qDebug() << "This is already the name of a group" ;
            valid = false;
        }
        for(int j = 0; j < i.value()->size(); j++){
            if(i.value()->at(j)->getPoint()->getName().compare(name.simplified(), Qt::CaseInsensitive) == 0){
                qDebug() << name << " already exists";
                valid = false;
            }
        }

        if(!valid){
            createPointWidget->getSaveBtn()->setEnabled(false);
            /// to explain the user why he cannot add its point as it is
            createPointWidget->getSaveBtn()->setToolTip("A point or group with this name already exists, please choose another name for your point");
            emit invalidName(TEXT_COLOR_WARNING, PointNameError::AlreadyExists);
            return;
        }
    }
    createPointWidget->getSaveBtn()->setToolTip("");
    createPointWidget->getSaveBtn()->setEnabled(true);
    emit invalidName(TEXT_COLOR_INFO, PointNameError::NoError);
}

void PointsController::updateBtnGroupPointsSlot(){
    displaySelectedGroup->getPointButtonGroup()->setGroup(displaySelectedGroup->getPointButtonGroup()->getGroupName(), points);
}

void PointsController::updateGroupButtonGroupSlot(){
    pointsLeftWidget->getGroupButtonGroup()->updateButtons(points);
}

void PointsController::sendMessageEditGroup(int code){
    pointsLeftWidget->setNameError(code);
    switch(code){
    case 0:
        emit setMessageTop(TEXT_COLOR_INFO, "To save this group press Enter or click the \"Save button\"");
        break;
    case 1:
        emit setMessageTop(TEXT_COLOR_INFO, "");
        break;
    case 2:
        emit setMessageTop(TEXT_COLOR_WARNING, "A group with the same name already exists, please choose another name for your group");
        break;
    default:
        qDebug() << "if you get here you probably forgot to implement the behavior for one or more error codes";
    }
}

void PointsController::enableButtonsPointsLeftWidget(QAbstractButton* button){
    qDebug() << "PointsLeftWidget::enableButtons called" << button->text();
    QString buttonTxt = button->text();

    points->setPixmapAll(PointView::PixmapType::NORMAL);
    emit resetPathPointViews();
    if(buttonTxt.compare(pointsLeftWidget->getLastCheckedId()) == 0){
        pointsLeftWidget->getGroupButtonGroup()->uncheck();
        pointsLeftWidget->getLastCheckedId() = "";
        pointsLeftWidget->disableButtons();
    } else {

        pointsLeftWidget->getLastCheckedId() = buttonTxt;
        pointsLeftWidget->getGroupButtonGroup()->setEditedGroupName(buttonTxt);
        pointsLeftWidget->getGroupButtonGroup()->getLayout()->removeWidget(pointsLeftWidget->getGroupButtonGroup()->getModifyEdit());
        pointsLeftWidget->getGroupButtonGroup()->getLayout()->addWidget(pointsLeftWidget->getGroupButtonGroup()->getModifyEdit());
        pointsLeftWidget->disableButtons();
        /// enables the minus buttonTxt
        pointsLeftWidget->getActionButtons()->getMinusButton()->setEnabled(true);
        if(points->isAGroup(buttonTxt))
            pointsLeftWidget->getActionButtons()->getMinusButton()->setToolTip("Click to remove the selected group");
        else
            pointsLeftWidget->getActionButtons()->getMinusButton()->setToolTip("Click to remove the selected point");

        /// enables the eye buttonTxt
        pointsLeftWidget->getActionButtons()->getGoButton()->setEnabled(true);
        if(points->isAGroup(buttonTxt))
            pointsLeftWidget->getActionButtons()->getGoButton()->setToolTip("Click to display the information of the selected group");
        else
            pointsLeftWidget->getActionButtons()->getGoButton()->setToolTip("Click to display the information of the selected point");

        /// enables the map buttonTxt
        pointsLeftWidget->getActionButtons()->getMapButton()->setCheckable(true);
        pointsLeftWidget->getActionButtons()->getMapButton()->setEnabled(true);

        if(points->isAGroup(buttonTxt)){
            if(points->isDisplayed(buttonTxt)){
                pointsLeftWidget->getActionButtons()->getMapButton()->setChecked(true);
                pointsLeftWidget->getActionButtons()->getMapButton()->setToolTip("Click to hide the selected group on the map");
            } else {
                pointsLeftWidget->getActionButtons()->getMapButton()->setChecked(false);
                pointsLeftWidget->getActionButtons()->getMapButton()->setToolTip("Click to display the selected group on the map");
            }
            /// changes the pointviews of all the points displayed in the group on the map
            for(int i = 0; i < points->getGroups()->value(buttonTxt)->size(); i++){
                QSharedPointer<PointView> pv = points->getGroups()->value(buttonTxt)->at(i);
                if(pv->isVisible())
                    pv->setPixmap(PointView::PixmapType::SELECTED);
            }
        } else {
            QSharedPointer<PointView> pv = points->findPointView(buttonTxt);
            if(pv->isVisible()){
                /// if the point is displayed, changes its pointview on the map
                points->findPointView(buttonTxt)->setPixmap(PointView::PixmapType::SELECTED);
                pointsLeftWidget->getActionButtons()->getMapButton()->setChecked(true);
                pointsLeftWidget->getActionButtons()->getMapButton()->setToolTip("Click to hide the selected point on the map");
            } else {
                pointsLeftWidget->getActionButtons()->getMapButton()->setChecked(false);
                pointsLeftWidget->getActionButtons()->getMapButton()->setToolTip("Click to display the selected point on the map");
            }
            /// if this point belongs to a path we also need to set the pixmap of the path point point view
            if(QSharedPointer<PointView> pathPv = points->findPathPointView(pv->getPoint()->getPosition().getX(), pv->getPoint()->getPosition().getY())){
                qDebug() << "PATH !";
                pathPv->setPixmap(PointView::PixmapType::SELECTED);
            } else {
                qDebug() << "NOT PATH";
            }
        }

        /// enables the edit buttonTxt
        pointsLeftWidget->getActionButtons()->getEditButton()->setEnabled(true);
        if(points->isAGroup(buttonTxt))
            pointsLeftWidget->getActionButtons()->getEditButton()->setToolTip("Click to modify the selected group");
        else
            pointsLeftWidget->getActionButtons()->getEditButton()->setToolTip("click to modify the selected point");
    }
}

int PointsController::checkGroupName(QString name){
    //qDebug() << "checking while creating" << name;
    pointsLeftWidget->getGroupNameEdit()->setText(Helper::formatName(name));
    name = name.simplified();
    //qDebug() << "name im testing" << name;
    if(!pointsLeftWidget->isCreatingGroup() && !name.compare(pointsLeftWidget->getGroupButtonGroup()->getEditedGroupName(), Qt::CaseInsensitive)){
        pointsLeftWidget->getSaveButton()->setToolTip("");
        qDebug() << "same name" << name;
        connect(pointsLeftWidget->getGroupNameEdit(), SIGNAL(clickSomewhere(QString)), pointsLeftWidget, SLOT(cancelCreationGroup()));
        return 0;
    }
    if(!name.compare("")){
        emit setMessageTop(TEXT_COLOR_NORMAL, "");
        return 1;
    }
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        bool valid(true);
        i.next();
        for(int j = 0; j < i.value()->size(); j++){
            if(!i.value()->at(j)->getPoint()->getName().compare(name.simplified(), Qt::CaseInsensitive)){
                qDebug() << name << "already exists as a point";
                valid = false;
            }
        }
        if(!name.compare(i.key(), Qt::CaseInsensitive)){
            qDebug() << "PointsLeftWidget::checkGroupName" << i.key() << "already exists";
            valid = false;

        }

        if(!valid){
            pointsLeftWidget->getSaveButton()->setToolTip("A group with the same name already exists, please choose another name for your group");
            pointsLeftWidget->getSaveButton()->setEnabled(false);
            connect(pointsLeftWidget->getGroupNameEdit(), SIGNAL(clickSomewhere(QString)), pointsLeftWidget, SLOT(cancelCreationGroup()));
            emit setMessageTop(TEXT_COLOR_WARNING, "A group or point with the same name already exists, please choose another name for your group");
            return 2;
        }
    }
    pointsLeftWidget->getSaveButton()->setToolTip("");
    pointsLeftWidget->getSaveButton()->setEnabled(true);
    disconnect(pointsLeftWidget->getGroupNameEdit(), SIGNAL(clickSomewhere(QString)), pointsLeftWidget, SLOT(cancelCreationGroup()));
    emit setMessageTop(TEXT_COLOR_INFO, "To save this group press Enter or click the \"Save button\"");
    return 0;
}
