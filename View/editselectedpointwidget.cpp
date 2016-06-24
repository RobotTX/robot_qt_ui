#include "editselectedpointwidget.h"
#include "View/pointview.h"
#include "View/pointsview.h"
#include "Model/point.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QMainWindow>
#include <QLineEdit>
#include <QDebug>
#include "Model/group.h"
#include <QComboBox>

EditSelectedPointWidget::EditSelectedPointWidget(QMainWindow* _parent, PointsView* _points):QWidget(_parent){
    parent = _parent;
    points = _points;

    //groupMenu = new GroupMenu(points->getPoints(), true);
    //groupMenu->displayReverse();
    groupBox = new QComboBox(this);
    /// to insert the groups in the box
    for(int i = 0; i < points->getPoints().count(); i++){
        groupBox->insertItem(points->getPoints().count()-1-i, points->getPoints().getGroups().at(i)->getName());
    }
    /// to set the default group as default
    groupBox->setCurrentIndex(0);

    layout = new QVBoxLayout(this);
    nameEdit = new QLineEdit(this);
    nameEdit->setStyleSheet ("text-align: left");
    layout->addWidget(nameEdit);

    posXLabel = new QLabel("X : ", this);
    layout->addWidget(posXLabel);

    posYLabel = new QLabel("Y : ", this);
    layout->addWidget(posYLabel);

    QHBoxLayout* grid = new QHBoxLayout();

    saveBtn = new QPushButton("Save", this);

    grid->addWidget(saveBtn);

    //layout->addWidget(groupMenu);
    layout->addWidget(groupBox);
    layout->addLayout(grid);

    connect(saveBtn, SIGNAL(clicked()), this, SLOT(saveEditSelecPointBtnEvent()));
    connect(nameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkPointName()));

    connect(groupBox, SIGNAL(activated(int)), this, SLOT(print(int)));
    qDebug() << groupBox->currentIndex();

    hide();
    setMaximumWidth(_parent->width()*4/10);
    setMinimumWidth(_parent->width()*4/10);
    layout->setAlignment(Qt::AlignTop);
}

void EditSelectedPointWidget::setSelectedPoint(PointView * const &_pointView, const bool isTemporary){
    _isTemporary = isTemporary;
    pointView = _pointView;
    nameEdit->setText(pointView->getPoint()->getName());
    posXLabel->setText("X : " + QString::number(pointView->getPoint()->getPosition().getX()));
    posYLabel->setText("Y : " + QString::number(pointView->getPoint()->getPosition().getY()));
}

void EditSelectedPointWidget::saveEditSelecPointBtnEvent(){
    qDebug() << "saveEditSelecPointBtnEvent called";
    if(isTemporary()){
        pointView->getPoint()->setName(nameEdit->text());

    }
    emit pointSaved();
}

void EditSelectedPointWidget::checkPointName(void) const {
    qDebug() << "checkPointName called";
    for(int i = 0; i < points->getPoints().count(); i++){
        std::shared_ptr<Group> group = points->getPoints().getGroups().at(i);
        for(int j = 0; j < group->count(); j++){
            if(!nameEdit->text().compare(group->getPoints().at(j)->getName())){
                qDebug() << nameEdit->text() << " already exists";
                saveBtn->setEnabled(false);
                saveBtn->setToolTip("A point with this name already exists, please choose another name for your point.");
                return;
            }
        }
    }
    saveBtn->setEnabled(true);
}

void EditSelectedPointWidget::updateGroupMenu(const Points& points){
    //groupMenu->updateList(points);
}

void EditSelectedPointWidget::print(int id) const {
    qDebug() << "id " << id;
}
