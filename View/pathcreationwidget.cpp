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


PathCreationWidget::PathCreationWidget(MainWindow *parent, const std::shared_ptr<Points> &_points): QWidget(parent), points(_points){
    layout = new QVBoxLayout(this);

    state = NO_STATE;

    actionButtons = new TopLeftMenu(this);
    actionButtons->getGoButton()->setEnabled(false);
    actionButtons->getMapButton()->setEnabled(false);
    actionButtons->getMinusButton()->setEnabled(false);
    actionButtons->getEditButton()->setEnabled(false);
    layout->addWidget(actionButtons);

    /// The menu which display the list of point to select
    pointsMenu = new QMenu(this);

    /// The list that displays the path points
    pathPointsList = new PathPointList(this);
    layout->addWidget(pathPointsList);

    QPushButton* cleanBtn = new QPushButton("Clean", this);
    layout->addWidget(cleanBtn);

    /// Cancel & save buttons
    QHBoxLayout* grid = new QHBoxLayout();
    QPushButton* cancelBtn = new QPushButton("Cancel", this);
    QPushButton* saveBtn = new QPushButton("Save", this);

    grid->addWidget(cancelBtn);
    grid->addWidget(saveBtn);

    layout->addLayout(grid);


    connect(actionButtons->getPlusButton(), SIGNAL(clicked()), this, SLOT(addPathPointByMenuSlot()));
    connect(actionButtons->getMinusButton(), SIGNAL(clicked()), this, SLOT(deletePathPointSlot()));
    connect(actionButtons->getEditButton(), SIGNAL(clicked()), this, SLOT(editPathPointSlot()));

    connect(pointsMenu, SIGNAL(triggered(QAction*)), this, SLOT(pointClicked(QAction*)));

    connect(pathPointsList, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(itemClicked(QListWidgetItem*)));
    connect(pathPointsList, SIGNAL(itemMovedSignal(QModelIndex, int, int, QModelIndex, int)), this, SLOT(itemMovedSlot(QModelIndex, int, int, QModelIndex, int)));

    connect(saveBtn, SIGNAL(clicked()), this, SLOT(savePathClicked()));
    connect(cancelBtn, SIGNAL(clicked()), parent, SLOT(cancelPathSlot()));
    connect(cleanBtn, SIGNAL(clicked()), this, SLOT(resetWidget()));


    hide();
    layout->setAlignment(Qt::AlignTop);
    layout->setContentsMargins(0, 0, 0, 0);
}

void PathCreationWidget::updateRobot(std::shared_ptr<Robot> robot){
    qDebug() << "PathCreationWidget::updateRobot called" << robot->getName();
    if (robot != NULL){
        for(int i = 0; i < robot->getPath().size(); i++){
            addPathPointSlot(robot->getPath().at(i)->getPoint().getName(),
                             robot->getPath().at(i)->getPoint().getPosition().getX(),
                             robot->getPath().at(i)->getPoint().getPosition().getY());
        }
    }
}

void PathCreationWidget::showEvent(QShowEvent* event){
    Q_UNUSED(event)
    //resetWidget();
    updatePointsList();
    show();
}

void PathCreationWidget::updatePointsList(void){
    qDebug() << "PathCreationWidget::updatePointsList called";
    pointsMenu->clear();

    /// We update the QMenu used to add/edit a permanent point
    QMapIterator<QString, std::shared_ptr<QVector<std::shared_ptr<PointView>>>> i(*(points->getGroups()));
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
        for(int k = 0; k < points->getGroups()->value(NO_GROUP_NAME)->count(); k++){
            pointsMenu->addAction(points->getGroups()->value(NO_GROUP_NAME)->at(k)->getPoint()->getName());
        }
    }
}

void PathCreationWidget::resetWidget(){
    qDebug() << "PathCreationWidget::resetWidget called";

    actionButtons->getMinusButton()->setEnabled(false);
    actionButtons->getEditButton()->setEnabled(false);

    pathPointsList->clear();
    state = NO_STATE;

    updatePointsList();
    emit resetPath();
}

void PathCreationWidget::itemClicked(QListWidgetItem* item){
    qDebug() << "PathCreationWidget::itemClicked called";
    /// When an item in the path point list is clicked, we select it

    actionButtons->getMinusButton()->setEnabled(true);
    actionButtons->getEditButton()->setEnabled(true);
    pathPointsList->setCurrentItem(item, QItemSelectionModel::Select);
}

void PathCreationWidget::itemMovedSlot(const QModelIndex& , int start, int , const QModelIndex& , int row){
    qDebug() << "PathCreationWidget::itemClicked called" << start << row;
    /// hen an item has been dragged in the path point list
    emit orderPathPointChanged(start, row);
}

void PathCreationWidget::savePathClicked(void){
    qDebug() << "PathCreationWidget::savePathClicked called";

    bool error = false;
    QString errorMsg = "";

    /// we check if we have path points
    if(pathPointsList->count() > 0){
        for(int i = 0; i < pathPointsList->count(); i++){
            PathPointCreationWidget* pathPointWidget = ((PathPointCreationWidget*) pathPointsList->itemWidget(pathPointsList->item(i)));

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
        emit savePath();
    }
}

void PathCreationWidget::addPathPointByMenuSlot(void){
    qDebug() << "PathCreationWidget::addPathPointByMenuSlot called";
    /// We had a point by clicking on the plus button
    state = CREATE;
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
    if(state == CREATE){
       qDebug() << "PathCreationWidget::pointClicked called to create a new path point" << action->text();
       addPathPointSlot(action->text(), pos.getX(), pos.getY());
    } else if(state == EDIT){
       qDebug() << "PathCreationWidget::pointClicked called to edit a path point into" << action->text();
       editPathPoint(action->text(), pos.getX(), pos.getY());
    } else {        qDebug() << "PathCreationWidget::pointClicked called while in NO_STATE" << action->text();
    }
}

void PathCreationWidget::addPathPointSlot(QString name, double x, double y){
    PathPointCreationWidget* pathPoint = new PathPointCreationWidget(pathPointsList->count(), name, x, y, this);
    connect(pathPoint, SIGNAL(saveEditSignal(PathPointCreationWidget*)), this, SLOT(saveEditSlot(PathPointCreationWidget*)));
    connect(pathPoint, SIGNAL(cancelEditSignal(PathPointCreationWidget*)), this, SLOT(cancelEditSlot(PathPointCreationWidget*)));
    connect(pathPoint, SIGNAL(actionChanged(int, int, QString)), this, SLOT(actionChangedSlot(int, int, QString)));

    /// We add the path point widget to the list
    QListWidgetItem* listWidgetItem = new QListWidgetItem(pathPointsList);
    listWidgetItem->setSizeHint(QSize(listWidgetItem->sizeHint().width(), WIDGET_HEIGHT));
    listWidgetItem->setBackgroundColor(QColor(255, 255, 255, 10));

    pathPointsList->addItem(listWidgetItem);
    pathPointsList->setItemWidget(listWidgetItem, pathPoint);

    if(pathPointsList->count() > 1){
        static_cast<PathPointCreationWidget*> (pathPointsList->itemWidget(pathPointsList->item(pathPointsList->count() - 2)))->displayActionWidget(true);
    }

    emit addPathPoint(name, x, y);
    state = NO_STATE;
}

void PathCreationWidget::deletePathPointSlot(void){
    qDebug() << "PathCreationWidget::deletePathPointSlot called";
    /// Delete the item and reset the widget
    deleteItem(pathPointsList->currentItem());
    if (pathPointsList->count()==0)
        resetWidget();
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
            emit deletePathPoint(id);
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

void PathCreationWidget::editPathPointSlot(void){
    qDebug() << "PathCreationWidget::editPathPoint called";
    int id = pathPointsList->row(pathPointsList->currentItem());
    state = EDIT;

    /// we edit the point only if the corresponding item is enabled which is the case if no other point is being edited
    if(pathPointsList->itemWidget(pathPointsList->currentItem())->isEnabled()){
        /// We get the edited pointView
        std::shared_ptr<PointView> pointView = points->getGroups()->value(PATH_GROUP_NAME)->at(id);
        qDebug() << "PathCreationWidget::editPathPoint"
                 << pointView->getPoint()->getName()
                 << pointView->getPoint()->getPosition().getX()
                 << pointView->getPoint()->getPosition().getY();


        if(pointView->getPoint()->getName().contains(PATH_POINT_NAME)){
            qDebug() << "PathCreationWidget::editPathPoint This is a temporary point";
            pathPointsList->setDragDropMode(QAbstractItemView::NoDragDrop);
            actionButtons->getPlusButton()->setEnabled(false);
            actionButtons->getMinusButton()->setEnabled(false);
            actionButtons->getEditButton()->setEnabled(false);

            PathPointCreationWidget* pathPointCreationWidget = static_cast<PathPointCreationWidget*> (pathPointsList->itemWidget(pathPointsList->currentItem()));
            pathPointCreationWidget->getEditWidget()->show();
            pathPointCreationWidget->getPathWidget()->hide();


            emit editTmpPathPoint(id, pointView->getPoint()->getName(),
                                  pointView->getPoint()->getPosition().getX(), pointView->getPoint()->getPosition().getY());
        } else {
            qDebug() << "PathCreationWidget::editPathPoint This is a permanent point";
            clicked();
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

    state = NO_STATE;

    emit saveEditPathPoint();
}

void PathCreationWidget::cancelEditSlot(PathPointCreationWidget* pathPointCreationWidget){
    qDebug() << "PathCreationWidget::cancelEditSlot called";
    actionButtons->getPlusButton()->setEnabled(true);
    actionButtons->getMinusButton()->setEnabled(true);
    actionButtons->getEditButton()->setEnabled(true);

    pathPointCreationWidget->getEditWidget()->hide();
    pathPointCreationWidget->getPathWidget()->show();
    pathPointsList->setDragDropMode(QAbstractItemView::InternalMove);

    state = NO_STATE;

    emit cancelEditPathPoint();
}

void PathCreationWidget::actionChangedSlot(int id, int action, QString waitTime){
    qDebug() << "PathCreationWidget::actionChangedSlot called" << id << waitTime;
    emit actionChanged(id, action, waitTime);
}


