#include "pathcreationwidget.h"
#include "View/pathpointcreationwidget.h"
#include "View/pathpointlist.h"
#include "View/toplayout.h"
#include "Model/pathpoint.h"
#include "Model/point.h"
#include "Model/robot.h"
#include "Model/group.h"
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
#include "View/spacewidget.h"

PathCreationWidget::PathCreationWidget(QMainWindow* parent, const Points &_points):QWidget(parent){
    layout = new QVBoxLayout(this);
    idPoint = 1;
    points = _points;
    selectedRobot = NULL;
    previousItem = NULL;
    editedPathPointCreationWidget = NULL;
    creatingNewPoint = false;
/*
    /// back button
    QPushButton* backBtn = new QPushButton(QIcon(":/icons/arrowLeft.png"),"Path", this);
    backBtn->setStyleSheet ("text-align: left");
    backBtn->setIconSize(parent->size()/10);
    layout->addWidget(backBtn);
    connect(backBtn, SIGNAL(clicked()), parent, SLOT(backPathCreation()));
*/

    QHBoxLayout* layoutRow1 = new QHBoxLayout();

    /// new button to add a pathpoint
    newBtn = new QPushButton(QIcon(":/icons/plus.png"), "", this);
    newBtn->setIconSize(parent->size()/10);
    layoutRow1->addWidget(newBtn);
    connect(newBtn, SIGNAL(clicked()), this, SLOT(addPathPoint()));

    /// delete button to delete a pathpoint
    supprBtn = new QPushButton(QIcon(":/icons/minus.png"), "", this);
    supprBtn->setCheckable(true);
    supprBtn->setIconSize(parent->size()/10);
    layoutRow1->addWidget(supprBtn);
    connect(supprBtn, SIGNAL(clicked()), this, SLOT(supprPathPoint()));

    /// edit button to edit a pathpoint
    editBtn = new QPushButton(QIcon(":/icons/edit.png"), "", this);
    editBtn->setCheckable(true);
    editBtn->setIconSize(parent->size()/10);
    layoutRow1->addWidget(editBtn);
    connect(editBtn, SIGNAL(clicked()), this, SLOT(editPathPoint()));

    layout->addLayout(layoutRow1);
    layout->setContentsMargins(0,0,0,0);


    /*QHBoxLayout* layoutRow2 = new QHBoxLayout();

    QPushButton* viewBtn = new QPushButton(QIcon(":/icons/eye.png"), "", this);
    viewBtn->setIconSize(parent->size()/10);
    layoutRow2->addWidget(viewBtn);
    //connect(viewBtn, SIGNAL(clicked()), this, SLOT(viewPathPoint()));

    QPushButton* mapBtn = new QPushButton(QIcon(":/icons/map.png"), "", this);
    mapBtn->setIconSize(parent->size()/10);
    layoutRow2->addWidget(mapBtn);
    //connect(mapBtn, SIGNAL(clicked()), this, SLOT(mapPathPoint()));

    layout->addLayout(layoutRow2);*/

    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    layout->addWidget(spaceWidget);

    /// The menu which display the list of point to select
    pointsMenu = new QMenu(this);
    connect(pointsMenu, SIGNAL(triggered(QAction*)), this, SLOT(pointClicked(QAction*)));

    for(int i = 0; i < points.getGroups().size(); i++){
        if(points.getGroups().at(i)->getName().compare("No group") == 0){
            for(int j = 0; j < points.getGroups().at(i)->getPoints().size(); j++){
                QString pointName = points.getGroups().at(i)->getPoints().at(j)->getName();
                pointsMenu->addAction(pointName);

                PointInfo pointInfo;
                pointInfo.name = pointName;
                pointInfo.posX = points.getGroups().at(i)->getPoints().at(j)->getPosition().getX();
                pointInfo.posY = points.getGroups().at(i)->getPoints().at(j)->getPosition().getY();
                pointInfos.push_back(pointInfo);
            }
        } else {
            QMenu *group = pointsMenu->addMenu("&" + points.getGroups().at(i)->getName());
            for(int j = 0; j < points.getGroups().at(i)->getPoints().size(); j++){
                QString pointName = points.getGroups().at(i)->getPoints().at(j)->getName();
                group->addAction(pointName);

                PointInfo pointInfo;
                pointInfo.name = pointName;
                pointInfo.posX = points.getGroups().at(i)->getPoints().at(j)->getPosition().getX();
                pointInfo.posY = points.getGroups().at(i)->getPoints().at(j)->getPosition().getY();
                pointInfos.push_back(pointInfo);
            }
        }
    }


    /// the list that displays the path points
    pathPointsList = new PathPointList(this);
    connect(pathPointsList, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(itemClicked(QListWidgetItem*)));
    connect(pathPointsList, SIGNAL(itemMovedSignal(int, int)), this, SLOT(itemMovedSlot(int, int)));

    layout->addWidget(pathPointsList);


    /// The save button
    QPushButton* saveBtn = new QPushButton("Save Path", this);
    layout->addWidget(saveBtn);
    connect(saveBtn, SIGNAL(clicked()), this, SLOT(saveNoExecPath()));
    connect(this, SIGNAL(pathSaved(bool)), parent, SLOT(pathSaved(bool)));


    /// The save button and play the path
    QPushButton* saveExecBtn = new QPushButton("Save and play Path", this);
    layout->addWidget(saveExecBtn);
    connect(saveExecBtn, SIGNAL(clicked()), this, SLOT(saveExecPath()));

    hide();
    layout->setAlignment(Qt::AlignTop);
    layout->setContentsMargins(0,0,0,0);
}

void PathCreationWidget::addPathPoint(void){
    qDebug() << "Add pathPoint" << idPoint;
    creatingNewPoint = true;
    clicked();
}

void PathCreationWidget::clicked(void){
    if(pointsMenu != NULL){
        pointsMenu->exec(QCursor::pos());
    }
}

void PathCreationWidget::pointClicked(QAction *action){
    qDebug() << "pointClicked called " << action->text();

    float posX = 0;
    float posY = 0;

    /// Get the pos of the point we selected in the menu
    for(int i = 0; i < pointInfos.size(); i++){
        if(pointInfos.at(i).name.compare(action->text()) == 0){
            posX = pointInfos.at(i).posX;
            posY = pointInfos.at(i).posY;
        }
    }

    /// Add the selected point to the path or edit the selected point
    if(creatingNewPoint){
        creatingNewPoint = false;
        addPathPoint(new Point(action->text(), posX, posY));

    } else {
        PathPointCreationWidget* pathPointCreationWidget = (PathPointCreationWidget*) pathPointsList->itemWidget(pathPointsList->currentItem());

        qDebug() << "Editing" << pathPointCreationWidget->getName() << "to" << action->text();
        pathPointCreationWidget->setName(action->text());
        pathPointCreationWidget->setPos(posX, posY);

        int id = pathPointCreationWidget->getId();
        Point point = pathPointCreationWidget->getPoint();

        pointList.replace(id-1, pathPointCreationWidget->getPoint());
        previousItem = NULL;
        pathPointsList->setCurrentItem(pathPointsList->currentItem(), QItemSelectionModel::Deselect);
        updatePointPainter();
    }
}

void PathCreationWidget::addPathPoint(Point* point){
    qDebug() << "Add pathPoint with point" << idPoint;

    /// We create a new widget to add to the list of path point widgets
    PathPointCreationWidget* pathPoint = new PathPointCreationWidget(idPoint, points, *point, this);

    initialisationPathPoint(pathPoint);

    pointList.push_back(*point);
    updatePointPainter();

    idPoint++;
}

void PathCreationWidget::initialisationPathPoint(PathPointCreationWidget* pathPoint){
    connect(pathPoint, SIGNAL(saveEditSignal(PathPointCreationWidget*)), this, SLOT(saveEditSlot(PathPointCreationWidget*)));

    /// We add the path point widget to the list
    QListWidgetItem* listWidgetItem = new QListWidgetItem(pathPointsList);
    listWidgetItem->setSizeHint(QSize(listWidgetItem->sizeHint().width(), WIDGET_HEIGHT));
    listWidgetItem->setBackgroundColor(QColor(255, 255, 255, 10));

    pathPointsList->addItem(listWidgetItem);
    pathPointsList->setItemWidget(listWidgetItem, pathPoint);

    if(pathPointsList->count() > 1){
        qDebug() << ((PathPointCreationWidget*) pathPointsList->itemWidget(pathPointsList->item(pathPointsList->count() - 2)))->getName();
        ((PathPointCreationWidget*) pathPointsList->itemWidget(pathPointsList->item(pathPointsList->count() - 2)))->displayActionWidget(true);
    }
}

void PathCreationWidget::itemClicked(QListWidgetItem* item){

    /// when we click on an item in the list we select/deselect it
    if(state == CheckState::NO_STATE){
        if(item == previousItem){
            qDebug() << "same same";
            previousItem = NULL;
            pathPointsList->setCurrentItem(item, QItemSelectionModel::Deselect);
        } else {
            qDebug() << "same but different";
            previousItem = item;
        }
    /// we delete it

    } else if(state == CheckState::SUPPR){
        qDebug() << "Ready to suppr";
        supprItem(item);
    /// or we edit it

    } else if(state == CheckState::EDIT){
        qDebug() << "Ready to edit";

        previousItem = item;
        editItem(item);
    }
}

void PathCreationWidget::supprPathPoint(){
    qDebug() << "supprPathPoint called";

    /// if the delete button is pressed, we toggle it if no path point is selected
    if(previousItem == NULL){
        if(supprBtn->isChecked()){
            state = CheckState::SUPPR;
            editBtn->setChecked(false);
        } else {
            state = CheckState::NO_STATE;
        }

    /// or we delete the selected path point
    } else {
        supprItem(pathPointsList->currentItem());
    }
}

void PathCreationWidget::editPathPoint(){
    qDebug() << "editPathPoint called";

    /// if the edit button is pressed, we toggle it if no path point is selected
    if(previousItem == NULL){
        if(editBtn->isChecked()){
            state = CheckState::EDIT;
            supprBtn->setChecked(false);
        /// or we edit the selected path point
        } else {
            state = CheckState::NO_STATE;
        }
    } else {
        if(editBtn->isChecked()){
            editItem(pathPointsList->currentItem());
        }

    }
}

void PathCreationWidget::saveNoExecPath(void){
    if(savePath()){
        emit pathSaved(false);
    } else {
        qDebug() << "Empty path";
    }
}

void PathCreationWidget::saveExecPath(void){
    if(savePath()){
        emit pathSaved(true);
    } else {
        qDebug() << "Empty path";
    }
}


bool PathCreationWidget::savePath(){
    qDebug() << "savePath called" << pathPointsList->count();
    bool error = false;
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
    }
}

void PathCreationWidget::resetWidget(){
    qDebug() << "resetWidget called";
    previousItem = NULL;
    editedPathPointCreationWidget = NULL;
    pathPointsList->setCurrentItem(pathPointsList->currentItem(), QItemSelectionModel::Deselect);

    pathPointsList->clear();
    idPoint = 1;
    supprBtn->setChecked(false);
    editBtn->setChecked(false);

    newBtn->setEnabled(true);
    supprBtn->setEnabled(true);
    editBtn->setEnabled(true);

    pointList.clear();
}

void PathCreationWidget::supprItem(QListWidgetItem* item){
    /// Pop up which ask to confirm the suppression of a path point
    QMessageBox msgBox;
    msgBox.setText("Are you sure you want to delete this point ?");
    msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::Cancel);
    int ret = msgBox.exec();
    int id;

    switch (ret) {
        case QMessageBox::Ok:

            id = pathPointsList->row(item);
            qDebug() << "Removing item" << id;
            pointList.remove(id);
            updatePointPainter();

            previousItem = NULL;
            pathPointsList->removeItemWidget(item);
            delete item;
            pathPointsList->refresh();
            idPoint--;
        break;
        case QMessageBox::Cancel:
            qDebug() << "Cancel was clicked";
        break;
        default:
            qDebug() << "Should never be reached";
        break;
    }
    supprBtn->setChecked(false);
    state = CheckState::NO_STATE;
    previousItem = NULL;
}

void PathCreationWidget::editItem(QListWidgetItem* item){
    /// Get the item to edit
    PathPointCreationWidget* pathPointWidget = (PathPointCreationWidget*) pathPointsList->itemWidget(item);

    /// if it's a temporary point we can move it
    if(pathPointWidget->isTemporary()){
        qDebug() << "Trying to edit a temporary point";
        int nbWidget = 0;
        /// Get the number of path point using the same point, because if there is only one,
        /// we move the point but if there is multiples, we create a new PointView to move
        /// and not edt the other path point using this point
        for(int i = 0; i < pathPointsList->count(); i++){
            PathPointCreationWidget* pathPointWidget2 = (PathPointCreationWidget*) pathPointsList->itemWidget(pathPointsList->item(i));
            if(abs(pathPointWidget2->getPosX() - pathPointWidget->getPosX()) < 0.01 &&
                    abs(pathPointWidget2->getPosY() - pathPointWidget->getPosY()) < 0.01)
                nbWidget++;
        }
        emit editTmpPathPoint(pathPointsList->row(item), new Point(pathPointWidget->getName(), pathPointWidget->getPosX(), pathPointWidget->getPosY()), nbWidget);
        state = CheckState::NO_STATE;
        pathPointWidget->displaySaveEditBtn(true, pathPointsList->count());
        pathPointsList->setDragDropMode(QAbstractItemView::NoDragDrop);
        editedPathPointCreationWidget = pathPointWidget;

        newBtn->setEnabled(false);
        supprBtn->setEnabled(false);
        editBtn->setEnabled(false);
    } else {
        qDebug() << "Trying to edit a permanent point";
        clicked();
    }

    editBtn->setChecked(false);
    state = CheckState::NO_STATE;
    previousItem = NULL;
}


void PathCreationWidget::updatePointPainter(){
    /*qDebug() << "\n";
    for(int i = 0; i < pointList.size(); i++){
        qDebug() << i << " : " << pointList.at(i).getName() << pointList.at(i).getPosition().getX() << pointList.at(i).getPosition().getY();
    }*/
    emit updatePathPointToPainter(&pointList);
}

void PathCreationWidget::hideEvent(QHideEvent *event){
    resetWidget();
    emit hidePathCreationWidget();
    QWidget::hideEvent(event);
}

void PathCreationWidget::itemMovedSlot(const int from, const int to){
    qDebug() << "itemMovedSlot called";
    Point point = pointList.takeAt(from);

    if(to >= pointList.size())
        pointList.push_back(point);
    else
        pointList.insert(to, point);

    updatePointPainter();
}

void PathCreationWidget::saveEditSlot(PathPointCreationWidget* pathPointCreationWidget){
    qDebug() << "saveEditSlot called";
    pathPointCreationWidget->displaySaveEditBtn(false, pathPointsList->count());
    pathPointsList->setDragDropMode(QAbstractItemView::InternalMove);

    newBtn->setEnabled(true);
    supprBtn->setEnabled(true);
    editBtn->setEnabled(true);
    emit saveEditPathPoint();
}

void PathCreationWidget::applySavePathPoint(float posX, float posY){
    qDebug() << "applySavePathPoint called" << posX << posY;
    editedPathPointCreationWidget->setPos(posX, posY);

    int id = editedPathPointCreationWidget->getId();
    pointList.replace(id-1, editedPathPointCreationWidget->getPoint());

    updatePointPainter();
    editedPathPointCreationWidget = NULL;
}

void PathCreationWidget::moveEditPathPoint(float posX, float posY){
    editedPathPointCreationWidget->setPos(posX, posY);

    int id = editedPathPointCreationWidget->getId();
    pointList.replace(id-1, editedPathPointCreationWidget->getPoint());

    updatePointPainter();
}
