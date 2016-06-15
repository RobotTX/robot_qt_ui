#include "pathcreationwidget.h"
#include "View/pathpointcreationwidget.h"
#include "View/pathpointlist.h"
#include "Model/pathpoint.h"
#include "Model/point.h"
#include "Model/robot.h"
#include "Model/robot.h"
#include <QListWidgetItem>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLineEdit>
#include <QDebug>
#include <QButtonGroup>
#include <QMessageBox>

PathCreationWidget::PathCreationWidget(QMainWindow* parent, const Points &_points){
    layout = new QVBoxLayout();
    idPoint = 1;
    points = _points;
    selectedRobot = NULL;
    previousItem = NULL;
    editedPathPointCreationWidget = NULL;

    /// back button
    QPushButton* backBtn = new QPushButton(QIcon(":/icons/arrowLeft.png"),"Path");
    backBtn->setStyleSheet ("text-align: left");
    backBtn->setIconSize(parent->size()/10);
    layout->addWidget(backBtn);
    connect(backBtn, SIGNAL(clicked()), parent, SLOT(backPathCreation()));


    QHBoxLayout* layoutRow1 = new QHBoxLayout();

    /// new button to add a pathpoint
    newBtn = new QPushButton(QIcon(":/icons/plus.png"), "");
    newBtn->setIconSize(parent->size()/10);
    layoutRow1->addWidget(newBtn);
    connect(newBtn, SIGNAL(clicked()), this, SLOT(addPathPoint()));

    /// delete button to delete a pathpoint
    supprBtn = new QPushButton(QIcon(":/icons/minus.png"), "");
    supprBtn->setCheckable(true);
    supprBtn->setIconSize(parent->size()/10);
    layoutRow1->addWidget(supprBtn);
    connect(supprBtn, SIGNAL(clicked()), this, SLOT(supprPathPoint()));

    /// edit button to edit a pathpoint
    editBtn = new QPushButton(QIcon(":/icons/edit.png"), "");
    editBtn->setCheckable(true);
    editBtn->setIconSize(parent->size()/10);
    layoutRow1->addWidget(editBtn);
    connect(editBtn, SIGNAL(clicked()), this, SLOT(editPathPoint()));

    layout->addLayout(layoutRow1);


    /*QHBoxLayout* layoutRow2 = new QHBoxLayout();

    QPushButton* viewBtn = new QPushButton(QIcon(":/icons/eye.png"), "");
    viewBtn->setIconSize(parent->size()/10);
    layoutRow2->addWidget(viewBtn);
    //connect(viewBtn, SIGNAL(clicked()), this, SLOT(viewPathPoint()));

    QPushButton* mapBtn = new QPushButton(QIcon(":/icons/map.png"), "");
    mapBtn->setIconSize(parent->size()/10);
    layoutRow2->addWidget(mapBtn);
    //connect(mapBtn, SIGNAL(clicked()), this, SLOT(mapPathPoint()));

    layout->addLayout(layoutRow2);*/


    /// the list that displays the path points
    pathPointsList = new PathPointList();
    connect(pathPointsList, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(itemClicked(QListWidgetItem*)));


    connect(pathPointsList, SIGNAL(itemMovedSignal(int, int)), this, SLOT(itemMovedSlot(int, int)));

    layout->addWidget(pathPointsList);


    QPushButton* saveBtn = new QPushButton("Save Path");
    layout->addWidget(saveBtn);
    connect(saveBtn, SIGNAL(clicked()), this, SLOT(savePath()));
    connect(this, SIGNAL(pathSaved()), parent, SLOT(pathSaved()));

    hide();
    layout->setAlignment(Qt::AlignTop);
    setLayout(layout);
}

PathCreationWidget::~PathCreationWidget(){
    delete layout;
    delete pathPointsList;
    delete newBtn;
    delete supprBtn;
    delete editBtn;
    delete selectedRobot;
    delete previousItem;
}

void PathCreationWidget::addPathPoint(void){
    qDebug() << "Add pathPoint" << idPoint;

    /// We create a new widget to add to the list of path point widgets
    PathPointCreationWidget* pathPoint = new PathPointCreationWidget(idPoint, points);
    connect(pathPoint, SIGNAL(pointSelected(PathPointCreationWidget*)), this, SLOT(pointSelected(PathPointCreationWidget*)));
    connect(pathPoint, SIGNAL(saveEditSignal(PathPointCreationWidget*)), this, SLOT(saveEditSlot(PathPointCreationWidget*)));

    /// We add the path point widget to the list
    QListWidgetItem* listWidgetItem = new QListWidgetItem();
    listWidgetItem->setSizeHint(QSize(listWidgetItem->sizeHint().width(), WIDGET_HEIGHT));
    listWidgetItem->setBackgroundColor(QColor(255, 255, 255, 10));
    pathPointsList->addItem(listWidgetItem);
    pathPointsList->setItemWidget(listWidgetItem, pathPoint);


    if(pathPointsList->count() > 1){
        qDebug() << ((PathPointCreationWidget*) pathPointsList->itemWidget(pathPointsList->item(pathPointsList->count() - 2)))->getName();
        ((PathPointCreationWidget*) pathPointsList->itemWidget(pathPointsList->item(pathPointsList->count() - 2)))->displayActionWidget(true);
    }

    pointList.push_back(Point());

    pathPoint->clicked();
    idPoint++;
}


void PathCreationWidget::addPathPoint(Point* point){
    qDebug() << "Add pathPoint with point" << idPoint;

    /// We create a new widget to add to the list of path point widgets
    PathPointCreationWidget* pathPoint = new PathPointCreationWidget(idPoint, points, *point);
    connect(pathPoint, SIGNAL(pointSelected(PathPointCreationWidget*)), this, SLOT(pointSelected(PathPointCreationWidget*)));
    connect(pathPoint, SIGNAL(saveEditSignal(PathPointCreationWidget*)), this, SLOT(saveEditSlot(PathPointCreationWidget*)));

    /// We add the path point widget to the list
    QListWidgetItem* listWidgetItem = new QListWidgetItem();
    listWidgetItem->setSizeHint(QSize(listWidgetItem->sizeHint().width(), WIDGET_HEIGHT));
    listWidgetItem->setBackgroundColor(QColor(255, 255, 255, 10));
    pathPointsList->addItem(listWidgetItem);
    pathPointsList->setItemWidget(listWidgetItem, pathPoint);


    if(pathPointsList->count() > 1){
        qDebug() << ((PathPointCreationWidget*) pathPointsList->itemWidget(pathPointsList->item(pathPointsList->count() - 2)))->getName();
        ((PathPointCreationWidget*) pathPointsList->itemWidget(pathPointsList->item(pathPointsList->count() - 2)))->displayActionWidget(true);
    }
    pointList.push_back(*point);
    updatePointPainter();

    idPoint++;
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
        editItem(item);

        previousItem = item;
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
        editBtn->setChecked(false);
        state = CheckState::NO_STATE;

        editItem(pathPointsList->currentItem());

    }
}

void PathCreationWidget::savePath(){
    qDebug() << "savePath called" << pathPointsList->count();
    bool error = false;

    /// we check if we have path points
    if(pathPointsList->count() > 0){
        for(int i = 0; i < pathPointsList->count(); i++){
            PathPointCreationWidget* pathPointWidget = ((PathPointCreationWidget*) pathPointsList->itemWidget(pathPointsList->item(i)));

            //qDebug() << pathPoint->getId() << pathPoint->getName();

            /// check if the path point has a name ( and not the default one)
            if(pathPointWidget->getName().compare("Select a point") != 0){

                /// check the action and if a number of time to wait has been set if needed
                if(pathPointWidget->getActionBtn()->text().compare("Human Action") == 0){

                } else if(pathPointWidget->getTimeEdit()->text().size() > 0
                          || i == (pathPointsList->count() - 1)){

                } else {
                    qDebug() << "Error point"<< i+1 <<" : Please select a time to wait";
                    error = true;
                }
            } else {
                qDebug() << "Error point"<< i+1 <<" : No point selected";
                error = true;
            }
        }
    } else {
        qDebug() << "Error : No point selected for the path";
        error = true;
    }

    ///if there is no error, we can save the path
    if(error){
        qDebug() << "Please make sure all the error(s) above has been fixed";
    } else {

        qDebug() << "No error, ready to save" << pointList.size() << pathPointsList->count();
        QVector<PathPoint*> path;
        for(int i = 0; i < pathPointsList->count(); i++){

            /// we try to get the point associated with the path point
            PathPointCreationWidget* pathPointWidget2 = ((PathPointCreationWidget*) pathPointsList->itemWidget(pathPointsList->item(i)));

            std::shared_ptr<Point> pointPtr = points.findPoint(pathPointWidget2->getName());

            if(pointPtr != NULL){
                Point point = *pointPtr;
                PathPoint::Action action;
                int waitTime = 0;
                //qDebug() << pathPoint->getId() << pathPoint->getName();

                if(pathPointWidget2->getActionBtn()->text().compare("Human Action") == 0){
                    action = PathPoint::HUMAN_ACTION;
                } else {
                    action = PathPoint::WAIT;
                    if(i != (pathPointsList->count() - 1))
                        waitTime = pathPointWidget2->getTimeEdit()->text().toInt();
                }

                path.push_back(new PathPoint(point, action, waitTime));

            } else {

                /// if there is no known point, it means we created a temporary one so we can
                /// create a pathpoint from the information of the path point
                /// ( and not from an existing point)
                qDebug() << "Temporary point : " << pathPointWidget2->getName() << pathPointWidget2->getPosX() << pathPointWidget2->getPosY();

                Point point(pathPointWidget2->getName(), pathPointWidget2->getPosX(), pathPointWidget2->getPosY());
                PathPoint::Action action;
                int waitTime = 0;
                qDebug() << pathPointWidget2->getName() << pathPointWidget2->getPosX() << pathPointWidget2->getPosY();

                if(pathPointWidget2->getActionBtn()->text().compare("Human Action") == 0){
                    action = PathPoint::HUMAN_ACTION;
                } else {
                    action = PathPoint::WAIT;
                    if(i != (pathPointsList->count() - 1))
                        waitTime = pathPointWidget2->getTimeEdit()->text().toInt();
                }
                path.push_back(new PathPoint(point, action, waitTime));
            }
        }
        qDebug() << "Path created for robot" << selectedRobot->getName();

        for(int i = 0; i < path.size(); i++){
            qDebug() << i << " : " << path.at(i)->getPoint().getName()
                     << path.at(i)->getPoint().getPosition().getX()
                     << path.at(i)->getPoint().getPosition().getY()
                     << path.at(i)->getWaitTime();
        }
        qDebug() << "\n";

        selectedRobot->setPath(path);
        emit pathSaved();
    }
}

void PathCreationWidget::resetWidget(){
    qDebug() << "resetWidget called";
    pathPointsList->clear();
    idPoint = 1;
    supprBtn->setChecked(false);
    editBtn->setChecked(false);
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
}

void PathCreationWidget::editItem(QListWidgetItem* item){
    PathPointCreationWidget* pathPointWidget = (PathPointCreationWidget*) pathPointsList->itemWidget(item);
    if(pathPointWidget->isTemporary()){
        qDebug() << "Trying to edit a temporary point";
        int nbWidget = 0;
        for(int i = 0; i < pathPointsList->count(); i++){
            PathPointCreationWidget* pathPointWidget2 = (PathPointCreationWidget*) pathPointsList->itemWidget(pathPointsList->item(i));
            if(abs(pathPointWidget2->getPosX() - pathPointWidget->getPosX()) < 0.01 &&
                    abs(pathPointWidget2->getPosY() - pathPointWidget->getPosY()) < 0.01)
                nbWidget++;
        }
        emit editTmpPathPoint(pathPointsList->row(item), new Point(pathPointWidget->getName(), pathPointWidget->getPosX(), pathPointWidget->getPosY()), nbWidget);
    } else {
        qDebug() << "Trying to edit a permanent point";
        pathPointWidget->clicked();
    }

    editBtn->setChecked(false);
    state = CheckState::NO_STATE;
    pathPointWidget->displaySaveEditBtn(true, pathPointsList->count());
    pathPointsList->setDragDropMode(QAbstractItemView::NoDragDrop);
    editedPathPointCreationWidget = pathPointWidget;

    newBtn->setEnabled(false);
    supprBtn->setEnabled(false);
    editBtn->setEnabled(false);
}

void PathCreationWidget::pointSelected(PathPointCreationWidget* pathPointCreationWidget){
    std::shared_ptr<Point> pointPtr = points.findPoint(pathPointCreationWidget->getName());
    int id = pathPointCreationWidget->getId();
    if(pointPtr != NULL){
        qDebug() << id << idPoint;
        if(id-1 >= pointList.size())
            pointList.push_back(*pointPtr);
        else
            pointList.replace(id-1, *pointPtr);
        updatePointPainter();
    }
}

void PathCreationWidget::updatePointPainter(){
    /*qDebug() << "\n";
    for(int i = 0; i < pointList.size(); i++){
        qDebug() << i << " : " << pointList.at(i).getName() << pointList.at(i).getPosition().getX() << pointList.at(i).getPosition().getY();
    }*/
    emit updatePathPointToPainter(&pointList);
}

void PathCreationWidget::hideEvent(QHideEvent *event){
    emit hidePathCreationWidget();
    newBtn->setEnabled(true);
    supprBtn->setEnabled(true);
    editBtn->setEnabled(true);
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
