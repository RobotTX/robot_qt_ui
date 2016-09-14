#include "pathcreationwidget.h"
#include "Controller/mainwindow.h"
#include "Model/points.h"
#include "Model/robot.h"
#include "View/topleftmenu.h"
#include <QDebug>
#include <QLabel>
#include <QMapIterator>
#include <QMenu>
#include <QListWidgetItem>
#include <QComboBox>
#include <QLineEdit>
#include "View/pathpointlist.h"
#include "Model/pathpoint.h"
#include "View/pathpointcreationwidget.h"
#include "View/custompushbutton.h"
#include <QLineEdit>
#include <QLabel>
#include <QGridLayout>
#include <QKeyEvent>
#include "View/customlineedit.h"

PathCreationWidget::PathCreationWidget(QWidget* parent, const QSharedPointer<Points> &_points, const QSharedPointer<Paths>& _paths, const bool associatedToRobot, const GraphicItemState _state):
    QWidget(parent), points(_points), paths(_paths), currentGroupName(""), currentPathName(""), state(_state)
{
    layout = new QVBoxLayout(this);

    checkState = NO_STATE;

    actionButtons = new TopLeftMenu(this);
    actionButtons->getGoButton()->setEnabled(false);
    actionButtons->getMapButton()->setEnabled(false);
    actionButtons->getMinusButton()->setEnabled(false);
    actionButtons->getEditButton()->setEnabled(false);
    layout->addWidget(actionButtons);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    /// to edit the name of a path if it is not a path associated to a robot

    nameLabel = new QLabel("Name :", this);
    nameLabel->setAlignment(Qt::AlignHCenter);
    nameEdit = new CustomLineEdit(this);
    if(associatedToRobot){
       nameEdit->hide();
       nameLabel->hide();
    }

    layout->addWidget(nameLabel);
    layout->addWidget(nameEdit);

    /// to check that the name given to the path is valid
    connect(nameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkPathName(QString)));

    ///////////////////////////////////////////////////////////////////////////////////////////////

    /// The menu which display the list of point to select
    pointsMenu = new QMenu(this);

    /// The list that displays the path points
    pathPointsList = new PathPointList(this);
    layout->addWidget(pathPointsList);


    /// Clean, cancel & save buttons
    QGridLayout* grid = new QGridLayout();
    cleanBtn = new CustomPushButton("Clean", this, CustomPushButton::ButtonType::LEFT_MENU, "center");
    grid->addWidget(cleanBtn, 0, 0);

    cancelBtn = new CustomPushButton("Cancel", this, CustomPushButton::ButtonType::LEFT_MENU, "center");

    saveBtn = new CustomPushButton("Save", this, CustomPushButton::ButtonType::LEFT_MENU, "center");

    grid->addWidget(cancelBtn, 1, 0);
    grid->addWidget(saveBtn, 1, 1);

    layout->addLayout(grid);

    connect(actionButtons->getPlusButton(), SIGNAL(clicked()), this, SLOT(addPathPointByMenuSlot()));
    connect(actionButtons->getMinusButton(), SIGNAL(clicked()), this, SLOT(deletePathPointSlot()));
    connect(actionButtons->getEditButton(), SIGNAL(clicked()), this, SLOT(editPathPointSlotRelay()));
    connect(this, SIGNAL(editPathPointSignal(GraphicItemState)), this, SLOT(editPathPointSlot(GraphicItemState)));
    connect(pointsMenu, SIGNAL(triggered(QAction*)), this, SLOT(pointClicked(QAction*)));

    connect(pathPointsList, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(itemClicked(QListWidgetItem*)));
    connect(pathPointsList, SIGNAL(itemMovedSignal(QModelIndex, int, int, QModelIndex, int)), this, SLOT(itemMovedSlot(QModelIndex, int, int, QModelIndex, int)));

    connect(saveBtn, SIGNAL(clicked()), this, SLOT(savePathClicked()));

    connect(cleanBtn, SIGNAL(clicked()), this, SLOT(resetWidgetRelaySlot()));

    connect(this, SIGNAL(resetWidgetSignal(GraphicItemState)), this, SLOT(resetWidget(GraphicItemState)));

    hide();
    layout->setAlignment(Qt::AlignTop);
    layout->setContentsMargins(0, 0, 0, 0);
}

void PathCreationWidget::updatePath(const QVector<QSharedPointer<PathPoint>>& _path){
    qDebug() << "PathCreationWidget::updatePath called with state" << state << "pathsize" << _path.size();
    pathPointsList->clear();
    for(int i = 0; i < _path.size(); i++){
        addPathPointSlot(_path.at(i)->getPoint().getName(),
                         _path.at(i)->getPoint().getPosition().getX(),
                         _path.at(i)->getPoint().getPosition().getY(), state);
    }
}

void PathCreationWidget::showEvent(QShowEvent* event){
    Q_UNUSED(event)
    updatePointsList();
    show();
}

void PathCreationWidget::updatePointsList(void){
    //qDebug() << "PathCreationWidget::updatePointsList called";
    pointsMenu->clear();

    /// We update the QMenu used to add/edit a permanent point
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();
        if(i.value() && i.key().compare(NO_GROUP_NAME)
             && i.key().compare(TMP_GROUP_NAME)
             && i.key().compare(PATH_GROUP_NAME)){
            if(i.value()->count() > 0){
                QMenu *group = pointsMenu->addMenu("&" + i.key());
                for(int j = 0; j < i.value()->count(); j++){
                    group->addAction(i.value()->at(j)->getPoint()->getName());
                }
            }
        }
    }

    if(points->getGroups()->value(NO_GROUP_NAME)){
        for(int k = 0; k < points->getGroups()->value(NO_GROUP_NAME)->count(); k++)
            pointsMenu->addAction(points->getGroups()->value(NO_GROUP_NAME)->at(k)->getPoint()->getName());
    }
}

void PathCreationWidget::resetWidget(GraphicItemState _state){
    //qDebug() << "PathCreationWidget::resetWidget called with state" << _state << "my state" << state;
    if(state == _state){
        //qDebug() << "PathCreationWidget::resetWidget called";

        actionButtons->getMinusButton()->setEnabled(false);
        actionButtons->getEditButton()->setEnabled(false);

        pathPointsList->clear();
        checkState = NO_STATE;

        updatePointsList();
        emit resetPath(state);
    }
}

void PathCreationWidget::itemClicked(QListWidgetItem* item){
    //qDebug() << "PathCreationWidget::itemClicked called";
    /// When an item in the path point list is clicked, we select it

    actionButtons->getMinusButton()->setEnabled(true);
    actionButtons->getEditButton()->setEnabled(true);
    pathPointsList->setCurrentItem(item, QItemSelectionModel::Select);
}

void PathCreationWidget::itemMovedSlot(const QModelIndex& , int start, int , const QModelIndex& , int row){
    //qDebug() << "PathCreationWidget::itemClicked called" << start << row;
    /// when an item has been dragged in the path point list
    emit orderPathPointChanged(start, row);
}

void PathCreationWidget::savePathClicked(void){
    qDebug() << "PathCreationWidget::savePathClicked called";

    bool error = false;
    QString errorMsg = "";

    /// we check if we have path points
    if(pathPointsList->count() > 0){
        for(int i = 0; i < pathPointsList->count(); i++){
            PathPointCreationWidget* pathPointWidget = static_cast<PathPointCreationWidget*> (pathPointsList->itemWidget(pathPointsList->item(i)));

            /// check the action and if a number of time to wait has been set if needed
            if(pathPointWidget->getAction()->currentText().compare("Human Action") != 0){

                if(pathPointWidget->getTimeEdit()->text().compare("") == 0
                          && i != (pathPointsList->count() - 1)){
                    errorMsg += "\tError point" + QString::number(i+1) + " : Please select a time to wait\n";
                    error = true;
                }
            }
        }
    } else {
        errorMsg += "\tError : No point selected for the path\n";
        error = true;
    }

    ///if there is no error, we can save the path
    if(error){
        QString msg = "Please make sure all the following error(s) have been fixed :\n" + errorMsg;
        emit setMessage(TEXT_COLOR_DANGER, msg);
    } else {
        qDebug() << "PathCreationWidget::savePathClicked No error, ready to save";
        emit savePath(state);
    }
}

void PathCreationWidget::addPathPointByMenuSlot(void){
    //qDebug() << "PathCreationWidget::addPathPointByMenuSlot called";
    /// We had a point by clicking on the plus button
    checkState = CREATE;
    clicked();
}

void PathCreationWidget::clicked(void){
    /// We triger the QMenu to display the list of permanent points
    qDebug() << (pointsMenu != NULL);
    if(pointsMenu != NULL){
        pointsMenu->exec(QCursor::pos());
    }
}

void PathCreationWidget::pointClicked(QAction *action){
    /// A permanent point has been selected on the QMenu so we add it to the list or edit the selected item
    Position pos = points->findPoint(action->text())->getPosition();
    if(checkState == CREATE){
       qDebug() << "PathCreationWidget::pointClicked called to create a new path point" << action->text();
       addPathPointSlot(action->text(), pos.getX(), pos.getY(), state);
    } else if(checkState == EDIT){
       qDebug() << "PathCreationWidget::pointClicked called to edit a path point into" << action->text();
       editPathPoint(action->text(), pos.getX(), pos.getY());
    } else
        qDebug() << "PathCreationWidget::pointClicked called while in NO_STATE" << action->text();
}

void PathCreationWidget::addPathPointSlot(QString name, double x, double y, GraphicItemState _state){
    if(state == _state){
        if(pathPointsList->count() > 0){
            PathPointCreationWidget* pathPoint = static_cast<PathPointCreationWidget*> (pathPointsList->itemWidget(pathPointsList->item(pathPointsList->count()-1)));
            Point helperPoint = Point(name, x, y);
            if(helperPoint.comparePos(pathPoint->getPosX(), pathPoint->getPosY())){
                qDebug() << "same point";
                return;
            }
        }

        PathPointCreationWidget* pathPoint = new PathPointCreationWidget(pathPointsList->count(), name, x, y, this);
        connect(pathPoint, SIGNAL(removePathPoint(PathPointCreationWidget*)), this, SLOT(deletePathPointWithCross(PathPointCreationWidget*)));
        connect(pathPoint, SIGNAL(saveEditSignal(PathPointCreationWidget*)), this, SLOT(saveEditSlot(PathPointCreationWidget*)));
        connect(pathPoint, SIGNAL(cancelEditSignal(PathPointCreationWidget*)), this, SLOT(cancelEditSlot(PathPointCreationWidget*)));
        connect(pathPoint, SIGNAL(actionChanged(int, int, QString)), this, SLOT(actionChangedSlot(int, int, QString)));

        /// We add the path point widget to the list
        QListWidgetItem* listWidgetItem = new QListWidgetItem(pathPointsList);
        listWidgetItem->setSizeHint(QSize(listWidgetItem->sizeHint().width(), WIDGET_HEIGHT));
        listWidgetItem->setBackgroundColor(QColor(255, 255, 255, 10));

        pathPointsList->addItem(listWidgetItem);
        pathPointsList->setItemWidget(listWidgetItem, pathPoint);

        if(pathPointsList->count() > 1)
            static_cast<PathPointCreationWidget*> (pathPointsList->itemWidget(pathPointsList->item(pathPointsList->count() - 2)))->displayActionWidget(true);

        emit addPathPoint(name, x, y);
        checkState = NO_STATE;
    }
}

void PathCreationWidget::deletePathPointSlot(){
    //qDebug() << "PathCreationWidget::deletePathPointSlot called with state" << state;

    /// Delete the item and reset the widget
    deleteItem(pathPointsList->currentItem());
    if (pathPointsList->count() == 0)
        resetWidget(state);

}
void PathCreationWidget::deleteItem(QListWidgetItem* item){
    /// Pop up which ask to confirm the suppression of a path point
    QMessageBox msgBox;
    msgBox.setText("Are you sure you want to delete this point ?");
    msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::Cancel);
    int ret = msgBox.exec();
    switch (ret) {
        case QMessageBox::Ok:
        {
            int id = pathPointsList->row(item);
            qDebug() << "PathCreationWidget::deleteItem Removing item" << id;
            pathPointsList->removeItemWidget(item);

            delete item;
            pathPointsList->refresh();
            emit deletePathPoint(id, state);
        }
        break;
        case QMessageBox::Cancel:
            qDebug() << "PathCreationWidget::deleteItem Cancel was clicked";
        break;
        default:
            qDebug() << "PathCreationWidget::deleteItem Should never be reached";
        break;
    }
}

void PathCreationWidget::editPathPointSlot(GraphicItemState _state){
    if(_state == state){
        //qDebug() << "PathCreationWidget::editPathPointSlot called";
        int id = pathPointsList->row(pathPointsList->currentItem());
        checkState = EDIT;

        /// we edit the point only if the corresponding item is enabled which is the case if no other point is being edited
        if(pathPointsList->itemWidget(pathPointsList->currentItem())->isEnabled()){
            /// We get the edited pointView
            QSharedPointer<PointView> pointView = points->getGroups()->value(PATH_GROUP_NAME)->at(id);
            /*
            qDebug() << "PathCreationWidget::editPathPointSlot"
                     << pointView->getPoint()->getName()
                     << pointView->getPoint()->getPosition().getX()
                     << pointView->getPoint()->getPosition().getY();

            */
            if(pointView->getPoint()->getName().contains(PATH_POINT_NAME)){
                qDebug() << "PathCreationWidget::editPathPointSlot This is a temporary point";
                pathPointsList->setDragDropMode(QAbstractItemView::NoDragDrop);
                actionButtons->getPlusButton()->setEnabled(false);
                actionButtons->getMinusButton()->setEnabled(false);
                actionButtons->getEditButton()->setEnabled(false);

                PathPointCreationWidget* pathPointCreationWidget = static_cast<PathPointCreationWidget*> (pathPointsList->itemWidget(pathPointsList->currentItem()));
                pathPointCreationWidget->getEditWidget()->show();
                pathPointCreationWidget->getPathWidget()->hide();


                emit editTmpPathPoint(id, pointView->getPoint()->getName(),
                                      pointView->getPoint()->getPosition().getX(), pointView->getPoint()->getPosition().getY(), state);
            } else {
                qDebug() << "PathCreationWidget::editPathPointSlot This is a permanent point";
                clicked();
            }
        }
    }
}
void PathCreationWidget::editPathPoint(QString name, double x, double y){
    qDebug() << "PathCreationWidget::editPathPoint called";
    PathPointCreationWidget* pathPointCreationWidget = static_cast<PathPointCreationWidget*> (pathPointsList->itemWidget(pathPointsList->currentItem()));
    pathPointCreationWidget->setPos(x, y);
    pathPointCreationWidget->setName(name);
    emit editPathPoint(pathPointsList->row(pathPointsList->currentItem()), name, x, y);
}

void PathCreationWidget::saveEditSlot(PathPointCreationWidget* pathPointCreationWidget){
    qDebug() << "PathCreationWidget::saveEditSlot called" << pathPointCreationWidget->getId();
    actionButtons->getPlusButton()->setEnabled(true);
    actionButtons->getMinusButton()->setEnabled(true);
    actionButtons->getEditButton()->setEnabled(true);

    Position pos = points->getGroups()->value(PATH_GROUP_NAME)->at(pathPointCreationWidget->getId())->getPoint()->getPosition();
    pathPointCreationWidget->setPos(pos.getX(), pos.getY());
    pathPointCreationWidget->getEditWidget()->hide();
    pathPointCreationWidget->getPathWidget()->show();
    pathPointsList->setDragDropMode(QAbstractItemView::InternalMove);

    checkState = NO_STATE;

    emit saveEditPathPoint(state);
}

void PathCreationWidget::cancelEditSlot(PathPointCreationWidget* pathPointCreationWidget){
    qDebug() << "PathCreationWidget::cancelEditSlot called";
    actionButtons->getPlusButton()->setEnabled(true);
    actionButtons->getMinusButton()->setEnabled(true);
    actionButtons->getEditButton()->setEnabled(true);

    pathPointCreationWidget->getEditWidget()->hide();
    pathPointCreationWidget->getPathWidget()->show();
    pathPointsList->setDragDropMode(QAbstractItemView::InternalMove);

    checkState = NO_STATE;

    emit cancelEditPathPoint(state);
}

void PathCreationWidget::actionChangedSlot(int id, int action, QString waitTime){
    qDebug() << "PathCreationWidget::actionChangedSlot called" << id << waitTime;
    emit actionChanged(id, action, waitTime);
}

void PathCreationWidget::checkPathName(const QString name){
    //qDebug() << "PathCreationWidget::checkPathName" << name.simplified();
    if(!name.simplified().compare("")){
        emit codeEditPath(0);
        //qDebug() << "PathCreatioNWidget::checkPathName The name of your path cannot be empty";
    }
    else if(!currentPathName.compare(name.simplified())){
        qDebug() << "same name";
        /// if the name simply has not changed the user can still save his path
         emit codeEditPath(2);
    }

    else {
        bool foundFlag(false);
        /// the found flag will have the value true after the call if the path has been found which means it already exists
        paths->getPath(currentGroupName, name.simplified(), foundFlag);
        if(foundFlag){
            //qDebug() << "PathCreatioNWidget::checkPathName Sorry there is already a path with the same name";
            emit codeEditPath(1);
        } else {
            //qDebug() << "PathCreatioNWidget::checkPathName nice this path does not exist yet !";
            emit codeEditPath(2);
        }
    }
}

void PathCreationWidget::keyPressEvent(QKeyEvent *event){
    /// if the name has not changed we valid (this is the job of the next function to check whether there is at least one point
    /// in the path and whether waiting times are filled

    if(!event->text().compare("\r")){
        qDebug() << "old path" << currentPathName << " new path" << nameEdit->text().simplified();
        if(!currentPathName.compare(nameEdit->text().simplified())){
            savePathClicked();
            return;
        }
        if(!nameEdit->text().simplified().compare(""))
            return;

        /// we check that the path name is valid (not taken and not empty)
        auto it = paths->getGroups()->find(currentGroupName);
        if(it != paths->getGroups()->end()){
            QSharedPointer<Paths::CollectionPaths> paths_ptr = it.value();
            QMapIterator<QString, QSharedPointer<Paths::Path> > it_paths(*paths_ptr);
            while(it_paths.hasNext()){
                it_paths.next();
                qDebug() << "pathCreationWidget::keyPressEvent, checking whether or not you can add this path" << it_paths.key();
                if(!it_paths.key().compare(nameEdit->text().simplified())){
                    qDebug() << "already a path named" << it_paths.key();
                    return;
                }
            }
        }
        savePathClicked();
    }
}

void PathCreationWidget::resetWidgetRelaySlot(){
    qDebug() << "resetwidgetsignal emitted";
    emit resetWidgetSignal(state);
}

void PathCreationWidget::editPathPointSlotRelay(){
    qDebug() << "editpathpointsignal emitted";
    emit editPathPointSignal(state);
}

void PathCreationWidget::deletePathPointWithCross(PathPointCreationWidget *pathPointCreationWidget){
    qDebug() << "PathCreationWidget::deletePathPointWithCross called" << pathPointCreationWidget->getId();
    /// Delete the item and reset the widget
    deleteItem(pathPointsList->item(pathPointCreationWidget->getId()));
    if(pathPointsList->count() == 0)
        resetWidget(state);
}
