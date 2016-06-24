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

EditSelectedPointWidget::EditSelectedPointWidget(QMainWindow* _parent, PointsView* _points){
    parent = _parent;
    points = _points;

    groupMenu = new GroupMenu(points->getPoints(), true);
    groupMenu->displayReverse();

    layout = new QVBoxLayout();
    nameEdit = new QLineEdit(_parent);
    nameEdit->setStyleSheet ("text-align: left");
    layout->addWidget(nameEdit);

    posXLabel = new QLabel("X : ");
    layout->addWidget(posXLabel);

    posYLabel = new QLabel("Y : ");
    layout->addWidget(posYLabel);

    QHBoxLayout* grid = new QHBoxLayout();

    saveBtn = new QPushButton("Save");

    //grid->addWidget(cancelBtn);
    grid->addWidget(saveBtn);

    layout->addWidget(groupMenu);
    layout->addLayout(grid);

    connect(saveBtn, SIGNAL(clicked()), this, SLOT(saveEditSelecPointBtnEvent()));
    connect(nameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkPointName()));

    hide();
    setMaximumWidth(_parent->width()*4/10);
    setMinimumWidth(_parent->width()*4/10);
    layout->setAlignment(Qt::AlignTop);
    setLayout(layout);
}

EditSelectedPointWidget::~EditSelectedPointWidget(){
    delete layout;
    delete pointView;
    delete nameEdit;
    delete posXLabel;
    delete posYLabel;
    delete points;
    delete saveBtn;
    delete parent;
    delete groupMenu;
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
    groupMenu->updateList(points);
}

