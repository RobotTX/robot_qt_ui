#include "pathcreationwidget.h"
#include "View/pathpointcreationwidget.h"
#include "View/pathpointlist.h"
#include "View/toplayout.h"
#include "Model/pathpoint.h"
#include "Model/point.h"
#include "Model/robot.h"
#include <QListWidgetItem>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLineEdit>
#include <QDebug>
#include <QButtonGroup>
#include <QMessageBox>
#include <QMenu>
#include <QComboBox>
#include <QLabel>
#include "View/spacewidget.h"
#include "topleftmenu.h"
#include "View/buttonmenu.h"
#include "View/pointview.h"
#include "View/mapview.h"

PathCreationWidget::PathCreationWidget(QMainWindow* parent, const std::shared_ptr<Points> &_points): QWidget(parent), idPoint(1), points(_points){
    layout = new QVBoxLayout(this);
    selectedRobot = NULL;
    previousItem = NULL;
    editedPathPointCreationWidget = NULL;
    creatingNewPoint = false;

    actionButtons = new TopLeftMenu(this);

    layout->addWidget(actionButtons);
    layout->addWidget(new QLabel("PathCreationWidget", this));
/*
    connect(actionButtons->getPlusButton(), SIGNAL(clicked(bool)), this, SLOT(addPathPoint()));
    connect(actionButtons->getMinusButton(), SIGNAL(clicked()), this, SLOT(supprPathPoint()));
    connect(actionButtons->getEditButton(), SIGNAL(clicked()), this, SLOT(editPathPoint()));

    /// The menu which display the list of point to select
    pointsMenu = new QMenu(this);
    connect(pointsMenu, SIGNAL(triggered(QAction*)), this, SLOT(pointClicked(QAction*)));

    for(int i = 0; i < points->getGroups().size(); i++){
        if(points->getGroups().at(i)->getName().compare(NO_GROUP_NAME) == 0){
            for(int j = 0; j < points->getGroups().at(i)->getPoints().size(); j++){
                QString pointName = points->getGroups().at(i)->getPoints().at(j)->getName();
                pointsMenu->addAction(pointName);

                PointInfo pointInfo;
                pointInfo.name = pointName;
                pointInfo.posX = points->getGroups().at(i)->getPoints().at(j)->getPosition().getX();
                pointInfo.posY = points->getGroups().at(i)->getPoints().at(j)->getPosition().getY();
                pointInfos.push_back(pointInfo);
            }
        } else {
            QMenu *group = pointsMenu->addMenu("&" + points->getGroups().at(i)->getName());
            for(int j = 0; j < points->getGroups().at(i)->getPoints().size(); j++){
                QString pointName = points->getGroups().at(i)->getPoints().at(j)->getName();
                group->addAction(pointName);

                PointInfo pointInfo;
                pointInfo.name = pointName;
                pointInfo.posX = points->getGroups().at(i)->getPoints().at(j)->getPosition().getX();
                pointInfo.posY = points->getGroups().at(i)->getPoints().at(j)->getPosition().getY();
                pointInfos.push_back(pointInfo);
            }
        }
    }


    /// the list that displays the path points
    pathPointsList = new PathPointList(this);
    connect(pathPointsList, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(itemClicked(QListWidgetItem*)));
    connect(pathPointsList, SIGNAL(itemMovedSignal(QModelIndex, int, int, QModelIndex, int)), this, SLOT(itemMovedSlot(QModelIndex, int, int, QModelIndex, int)));

    layout->addWidget(pathPointsList);


    QVBoxLayout* bottomLayout = new QVBoxLayout();

    /// The save button
    QPushButton* saveBtn = new QPushButton("Save Path", this);

    bottomLayout->addWidget(saveBtn);

    connect(saveBtn, SIGNAL(clicked()), this, SLOT(saveNoExecPath()));
    connect(this, SIGNAL(pathSaved(bool)), parent, SLOT(pathSaved(bool)));

    /// The save button and play the path
    QPushButton* saveExecBtn = new QPushButton("Save and play Path", this);
    bottomLayout->addWidget(saveExecBtn);
    connect(saveExecBtn, SIGNAL(clicked()), this, SLOT(saveExecPath()));
    layout->addLayout(bottomLayout);

    hide();

    layout->setAlignment(Qt::AlignTop);
    layout->setContentsMargins(0, 0, 0, 0);
    */
}

void PathCreationWidget::addPathPoint(void){
    qDebug() << " Add pathPoint" << idPoint;
    /*creatingNewPoint = true;
    clicked();*/
}

void PathCreationWidget::clicked(void){
    /*if(pointsMenu != NULL){
        pointsMenu->exec(QCursor::pos());
    }*/
}

void PathCreationWidget::pointClicked(QAction *action){
   //pointClicked( action->text());
}

void PathCreationWidget::pointClicked(QString name){

    qDebug() << "pointClicked called " << name;

    /*float posX = 0;
    float posY = 0;

    /// Get the pos of the point we selected in the menu
    for(int i = 0; i < pointInfos.size(); i++){
        if(pointInfos.at(i).name.compare(name) == 0){
            posX = pointInfos.at(i).posX;
            posY = pointInfos.at(i).posY;
        }
    }

    /// Add the selected point to the path or edit the selected point
    if(creatingNewPoint){
        creatingNewPoint = false;
        addPathPoint(new Point(name, posX, posY));

    } else {
        PathPointCreationWidget* pathPointCreationWidget = static_cast<PathPointCreationWidget*> (pathPointsList->itemWidget(pathPointsList->currentItem()));

        qDebug() << "Editing" << pathPointCreationWidget->getName() << "to" << name;
        /// to update the path known to the mapview
        changePermanentPoint(pathPointCreationWidget->getName(), name);

        pathPointCreationWidget->setName(name);
        pathPointCreationWidget->setPos(posX, posY);

        int id = pathPointCreationWidget->getId();
        Point point = pathPointCreationWidget->getPoint();

        pointList.replace(id-1, pathPointCreationWidget->getPoint());
        previousItem = NULL;

        /// to update the path known to the mapview
        changePermanentPoint(pathPointCreationWidget->getName(), name);
        updatePointPainter();
    }*/
}

void PathCreationWidget::addPathPoint(Point* point){
    qDebug() << "Add pathPoint with point" << idPoint;

    /*/// we first check that the last point of our path is not the same one
    if(!pointList.last().comparePos(point->getPosition())){
        /// We create a new widget to add to the list of path point widgets
        PathPointCreationWidget* pathPoint = new PathPointCreationWidget(idPoint, *points, *point, this);
        initialisationPathPoint(pathPoint);
        pointList.push_back(*point);
        updatePointPainter();
        emit addMapPathPoint(point);
        for(int i = 0; i < pointList.size(); i++)
            qDebug() << pointList.at(i).getName();
        idPoint++;
    } else
        qDebug() << "This point is identical to the last one";*/
}

void PathCreationWidget::initialisationPathPoint(PathPointCreationWidget* pathPoint){
    /*connect(pathPoint, SIGNAL(saveEditSignal(PathPointCreationWidget*)), this, SLOT(saveEditSlot(PathPointCreationWidget*)));

    /// We add the path point widget to the list
    QListWidgetItem* listWidgetItem = new QListWidgetItem(pathPointsList);
    listWidgetItem->setSizeHint(QSize(listWidgetItem->sizeHint().width(), WIDGET_HEIGHT));
    listWidgetItem->setBackgroundColor(QColor(255, 255, 255, 10));

    pathPointsList->addItem(listWidgetItem);
    pathPointsList->setItemWidget(listWidgetItem, pathPoint);

    if(pathPointsList->count() > 1){
        qDebug() << ((PathPointCreationWidget*) pathPointsList->itemWidget(pathPointsList->item(pathPointsList->count() - 2)))->getName();
        ((PathPointCreationWidget*) pathPointsList->itemWidget(pathPointsList->item(pathPointsList->count() - 2)))->displayActionWidget(true);
    }*/
}

void PathCreationWidget::itemClicked(QListWidgetItem* item){
    /*qDebug() << "item click in list path";
    actionButtons->getMinusButton()->setEnabled(true);
    actionButtons->getEditButton()->setEnabled(true);
    pathPointsList->setCurrentItem(item, QItemSelectionModel::Select);*/
}

void PathCreationWidget::supprPathPoint(){
    qDebug() << "supprPathPoint called";
/*
    supprItem(pathPointsList->currentItem());
    if (pathPointsList->count()==0)
        resetWidget();
*/
}

void PathCreationWidget::editPathPoint(){
    qDebug() << "editPathPoint called";
/*
    /// we edit the point only if the corresponding item is enabled which is the case if no other point is being edited
    if(pathPointsList->itemWidget(pathPointsList->currentItem())->isEnabled()){
        state = CheckState::EDIT;
        editItem(pathPointsList->currentItem());
        if  ( pathPointsList->selectedItems().count() == 0 )
        {
            actionButtons->getMinusButton()->setEnabled(false);
            actionButtons->getEditButton()->setEnabled(false);
        }
        if(!static_cast<PathPointCreationWidget*> ((pathPointsList->itemWidget(pathPointsList->currentItem())))->getPoint().isPermanent()){
            for(int i = 0; i < pathPointsList->count(); i++){
                if(!pathPointsList->item(i)->isSelected()){
                    qDebug() << "yo";
                    static_cast<PathPointCreationWidget*> (pathPointsList->itemWidget(pathPointsList->item(i)))->setEnabled(false);
                }
            }
        }
    }*/
}

void PathCreationWidget::saveNoExecPath(void){
    /*if(savePath()){
        emit pathSaved(false);
    } else {
        qDebug() << "Empty path";
    }*/
}

void PathCreationWidget::saveExecPath(void){
    /*if(savePath()){
        emit pathSaved(true);
    } else {
        qDebug() << "Empty path";
    }*/
}


bool PathCreationWidget::savePath(){
    //qDebug() << "savePath called" << pathPointsList->count();
    /*bool error = false;
    QString errorMsg = "";

    /// we check if we have path points
    if(pathPointsList->count() > 0){
        for(int i = 0; i < pathPointsList->count(); i++){
            PathPointCreationWidget* pathPointWidget = ((PathPointCreationWidget*) pathPointsList->itemWidget(pathPointsList->item(i)));

            qDebug() << pathPointWidget->getId() << pathPointWidget->getName()
                     << pathPointWidget->getPoint().getPosition().getX()
                     << pathPointWidget->getPoint().getPosition().getY();

            /// check if the path point has a name ( and not the default one)
            if(pathPointWidget->getName().compare("Select a point") != 0){

                /// check the action and if a number of time to wait has been set if needed
                if(pathPointWidget->getAction()->currentText().compare("Human Action") == 0){

                } else if(pathPointWidget->getTimeEdit()->text().size() > 0
                          || i == (pathPointsList->count() - 1)){

                } else {
                    errorMsg += "\tError point" + QString::number(i+1) + " : Please select a time to wait\n";
                    error = true;
                }
            } else {
                errorMsg += "\tError point" + QString::number(i+1) + " : No point selected\n";
                error = true;
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
        return false;
    } else {
        qDebug() << "No error, ready to save" << pointList.size() << pathPointsList->count();
        std::vector<std::shared_ptr<PathPoint>> path;
        for(int i = 0; i < pointList.size(); i++){

            /// we try to get the point associated with the path point
            PathPointCreationWidget* pathPointWidget2 = ((PathPointCreationWidget*) pathPointsList->itemWidget(pathPointsList->item(i)));

            PathPoint::Action action;
            int waitTime = 0;

            if(pathPointWidget2->getAction()->currentText().compare("Human Action") == 0){
                action = PathPoint::HUMAN_ACTION;
            } else {
                action = PathPoint::WAIT;
                if(i != (pathPointsList->count() - 1))
                    waitTime = pathPointWidget2->getTimeEdit()->text().toInt();
            }

            QString name = pointList.at(i).getName();
            if(pointList.at(i).getName().compare("tmpPoint") == 0){
                name = QString::number(pointList.at(i).getPosition().getX(),'f', 1) + "; " +
                        QString::number(pointList.at(i).getPosition().getY(),'f', 1);
            }
            Point point(name, pointList.at(i).getPosition().getX(), pointList.at(i).getPosition().getY());
            path.push_back(std::shared_ptr<PathPoint>(new PathPoint(point, action, waitTime)));
        }
        qDebug() << "Path created for robot" << selectedRobot->getName();

        for(size_t i = 0; i < path.size(); i++){
            qDebug() << i << " : " << path.at(i)->getPoint().getName()
                     << path.at(i)->getPoint().getPosition().getX()
                     << path.at(i)->getPoint().getPosition().getY()
                     << path.at(i)->getWaitTime();
        }
        qDebug() << "\n";

        selectedRobot->setPath(path);
        return true;
    }*/
}

void PathCreationWidget::resetWidget(){
    qDebug() << "pathcreationwidget resetWidget called";
    /*previousItem = NULL;
    editedPathPointCreationWidget = NULL;
    pathPointsList->setCurrentItem(pathPointsList->currentItem(), QItemSelectionModel::Deselect);

    pathPointsList->clear();
    idPoint = 1;

    actionButtons->uncheckAll();

    actionButtons->disableAll();
    actionButtons->getPlusButton()->setEnabled(true);

    pointList.clear();*/
}

void PathCreationWidget::supprItem(QListWidgetItem* item){
    /// Pop up which ask to confirm the suppression of a path point
    /*QMessageBox msgBox;
    msgBox.setText("Are you sure you want to delete this point ?");
    msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::Cancel);
    int ret = msgBox.exec();
    int id;
    Point pt;
    switch (ret) {
        case QMessageBox::Ok:

            id = pathPointsList->row(item);
            qDebug() << "Removing item" << id;
            pt = pointList.at(id);

            pointList.remove(id);
            updatePointPainter();

            previousItem = NULL;
            pathPointsList->removeItemWidget(item);

            delete item;
            pathPointsList->refresh();
            idPoint--;
            emit deletePointView(pt);

        break;
        case QMessageBox::Cancel:
            qDebug() << "Cancel was clicked";
        break;
        default:
            qDebug() << "Should never be reached";
        break;
    }
    actionButtons->getMinusButton()->setChecked(false);
    state = CheckState::NO_STATE;
    previousItem = NULL;*/
}

void PathCreationWidget::editItem(QListWidgetItem* item){
    qDebug() << "editItem " ;
    /// Get the item to edit
    /*PathPointCreationWidget* pathPointWidget = static_cast<PathPointCreationWidget*> (pathPointsList->itemWidget(item));

    /// if it's a temporary point we can move it
    if(!pathPointWidget->getPoint().isPermanent()){
        qDebug() << "Trying to edit a temporary point";
        int nbWidget = 0;
        /// Get the number of path point using the same point, because if there is only one,
        /// we move the point but if there is multiples, we create a new PointView to move
        /// and not edt the other path point using this point
        for(int i = 0; i < pathPointsList->count(); i++){
            PathPointCreationWidget* pathPointWidget2 = static_cast<PathPointCreationWidget*> (pathPointsList->itemWidget(pathPointsList->item(i)));
            if(abs(pathPointWidget2->getPosX() - pathPointWidget->getPosX()) < 0.01 &&
                    abs(pathPointWidget2->getPosY() - pathPointWidget->getPosY()) < 0.01)
                nbWidget++;
        }
        qDebug() << "found" << nbWidget << "widgets";
        emit editTmpPathPoint(pathPointsList->row(item), new Point(pathPointWidget->getName(), pathPointWidget->getPosX(), pathPointWidget->getPosY()), nbWidget);
        state = CheckState::NO_STATE;
        pathPointWidget->displaySaveEditBtn(true, pathPointsList->count());
        pathPointsList->setDragDropMode(QAbstractItemView::NoDragDrop);
        editedPathPointCreationWidget = pathPointWidget;

    } else {
        qDebug() << "Trying to edit a permanent point";
        clicked();
        pathPointsList->setEnabled(true);
    }

    state = CheckState::NO_STATE;
    previousItem = NULL;*/
}


void PathCreationWidget::updatePointPainter(const bool save){
    qDebug() << "pathcreationwidget updatepointpainer called";
    //emit updatePathPointToPainter(pointList, save);
}

void PathCreationWidget::hideEvent(QHideEvent *event){
    /*resetWidget();
    emit hidePathCreationWidget();
    QWidget::hideEvent(event);*/
}

void PathCreationWidget::itemMovedSlot(const QModelIndex& , int start, int , const QModelIndex& , int row){
/*
    Point point = pointList.takeAt(start);

    if(row > pointList.size())  pointList.push_back(point);

    else {
        if(start < row)         pointList.insert(row-1, point);
        else                    pointList.insert(row, point);
    }

    /// to notify the mapView that the order of the points has changed
    emit orderPointsChanged(start, row);

    updatePointPainter();*/
}

void PathCreationWidget::saveEditSlot(PathPointCreationWidget* pathPointCreationWidget){
    qDebug() << "saveEditSlot called";
    /*pathPointCreationWidget->displaySaveEditBtn(false, pathPointsList->count());
    pathPointsList->setDragDropMode(QAbstractItemView::InternalMove);

    actionButtons->getPlusButton()->setEnabled(true);
    actionButtons->getMinusButton()->setEnabled(true);
    actionButtons->getEditButton()->setEnabled(true);
    state = CheckState::NO_STATE;

    emit saveEditPathPoint();*/
}

void PathCreationWidget::applySavePathPoint(const float posX, const float posY, const bool save){
    Q_UNUSED(save)
    qDebug() << "applySavePathPoint called" << posX << posY;
    /*editedPathPointCreationWidget->setPos(posX, posY);

    int id = editedPathPointCreationWidget->getId();
    pointList.replace(id-1, editedPathPointCreationWidget->getPoint());

    updatePointPainter();
    editedPathPointCreationWidget = NULL;
    for(int i = 0; i < pathPointsList->count(); i++)
        pathPointsList->itemWidget(pathPointsList->item(i))->setEnabled(true);*/
}

void PathCreationWidget::moveEditPathPoint(float posX, float posY){
    /*editedPathPointCreationWidget->setPos(posX, posY);

    int id = editedPathPointCreationWidget->getId();

    pointList.replace(id-1, editedPathPointCreationWidget->getPoint());

    updatePointPainter();*/
}


void PathCreationWidget::showEvent(QShowEvent *){
    /*resetWidget();
    actionButtons->disableAll();
    actionButtons->getPlusButton()->setEnabled(true);
    /// updates the edit path menu
    updateMenu();
    updateList();
    qDebug() << "show Path Creation Widget";
*/
}

void PathCreationWidget::updateMenu(){
    /*pointsMenu->clear();
    for(int i = 0; i < points->getGroups().size(); i++){
        if(points->getGroups().at(i)->getName().compare(NO_GROUP_NAME) == 0){
            for(int j = 0; j < points->getGroups().at(i)->getPoints().size(); j++){
                QString pointName = points->getGroups().at(i)->getPoints().at(j)->getName();
                pointsMenu->addAction(pointName);

                PointInfo pointInfo;
                pointInfo.name = pointName;
                pointInfo.posX = points->getGroups().at(i)->getPoints().at(j)->getPosition().getX();
                pointInfo.posY = points->getGroups().at(i)->getPoints().at(j)->getPosition().getY();
                pointInfos.push_back(pointInfo);
            }
        } else {
            QMenu *group = pointsMenu->addMenu("&" + points->getGroups().at(i)->getName());
            for(int j = 0; j < points->getGroups().at(i)->getPoints().size(); j++){
                QString pointName = points->getGroups().at(i)->getPoints().at(j)->getName();
                group->addAction(pointName);

                PointInfo pointInfo;
                pointInfo.name = pointName;
                pointInfo.posX = points->getGroups().at(i)->getPoints().at(j)->getPosition().getX();
                pointInfo.posY = points->getGroups().at(i)->getPoints().at(j)->getPosition().getY();
                pointInfos.push_back(pointInfo);
            }
        }
    }*/
}

void PathCreationWidget::updateList(){
    qDebug() << "update list called";
    /*std::shared_ptr<Point> pt = std::shared_ptr<Point>(new Point());

    if (selectedRobot != NULL)
    {
        for (size_t i = 0; i < selectedRobot->getPath().size(); i++)
        {

            *pt = selectedRobot->getPath().at(i)->getPoint();
            if (pt->getName().contains(';'))
                emit addPointEditPath(*pt);
            else
                addPathPoint(&(*pt));
            int action =  selectedRobot->getPath().at(i)->getAction();
            if (action == PathPoint::Action::WAIT)
                pathPointsList->update(i, 0, selectedRobot->getPath().at(i)->getWaitTime());
            else
                pathPointsList->update(i, 1);
        }
    }*/
}

