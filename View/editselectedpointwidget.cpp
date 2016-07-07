#include "editselectedpointwidget.h"
#include "View/pointview.h"
#include "View/pointsview.h"
#include "Model/point.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QMainWindow>
#include <QLineEdit>
#include <QDebug>
#include "Model/group.h"
#include <QComboBox>
#include "View/spacewidget.h"
#include <QKeyEvent>

EditSelectedPointWidget::EditSelectedPointWidget(QMainWindow* _parent, PointsView* _points):QWidget(_parent){
    parent = _parent;
    points = _points;

    separator = new SpaceWidget(SpaceWidget::HORIZONTAL, this);

    ///                     PLUS MINUS AND EDIT BUTTONS

    topButtonsLayout = new QHBoxLayout();

    plusButton = new QPushButton(QIcon(":/icons/plus.png"),"", this);
    plusButton->setIconSize(_parent->size()/10);
    plusButton->setToolTip("Click this button if you want to save this point permanently");

    minusButton = new QPushButton(QIcon(":/icons/minus.png"),"", this);
    minusButton->setIconSize(_parent->size()/10);
    minusButton->setEnabled(false);

    editButton = new QPushButton(QIcon(":/icons/edit.png"),"", this);
    editButton->setIconSize(_parent->size()/10);
    editButton->setEnabled(false);

    topButtonsLayout->addWidget(plusButton);
    topButtonsLayout->addWidget(minusButton);
    topButtonsLayout->addWidget(editButton);

    ///                    EYE AND MAP BUTTONS

    eyeMapLayout = new QHBoxLayout();

    eyeButton = new QPushButton(QIcon(":/icons/eye.png"), "", this);
    eyeButton->setIconSize(_parent->size()/10);
    eyeButton->setEnabled(false);

    mapButton = new QPushButton(QIcon(":/icons/map.png"),"", this);
    mapButton->setIconSize(_parent->size()/10);
    mapButton->setEnabled(false);

    eyeMapLayout->addWidget(eyeButton);
    eyeMapLayout->addWidget(mapButton);

    ///                  add buttons to the main layout

    layout = new QVBoxLayout(this);
    layout->addLayout(topButtonsLayout);
    layout->addLayout(eyeMapLayout);

    layout->addWidget(separator);

    nameEdit = new QLineEdit(this);
    nameEdit->setStyleSheet ("text-align: left");
    nameEdit->setReadOnly(true);
    nameEdit->setStyleSheet("* { background-color: rgba(255, 0, 0, 0); }");
    nameEdit->setAutoFillBackground(true);
    nameEdit->setFrame(false);
    layout->addWidget(nameEdit);

    posXLabel = new QLabel("X : ", this);
    layout->addWidget(posXLabel);

    posYLabel = new QLabel("Y : ", this);
    layout->addWidget(posYLabel);

    ///                                  ADD GROUP LABEL AND QComboBox

    groupLayout = new QHBoxLayout();

    groupLabel = new QLabel("Group : ", this);
    groupLabel->hide();
    groupBox = new QComboBox(this);

    /// to insert the groups in the box
    for(int i = 0; i < points->getPoints()->count(); i++){
        groupBox->insertItem(points->getPoints()->count()-1-i, points->getPoints()->getGroups().at(i)->getName());
    }

    /// to set the default group as default
    groupBox->setCurrentIndex(0);
    groupBox->setItemIcon(0, QIcon(":/icons/tick.png"));
    groupBox->hide();

    groupLayout->addWidget(groupLabel);
    groupLayout->addWidget(groupBox);

    ///                                   ADD CANCEL AND SAVE BUTTONS

    cancelSaveLayout = new QHBoxLayout();


    saveBtn = new QPushButton("Save", this);
    cancelBtn = new QPushButton("Cancel", this);
    cancelSaveLayout->addWidget(cancelBtn);
    cancelSaveLayout->addWidget(saveBtn);
    saveBtn->hide();
    cancelBtn->hide();

    layout->addLayout(groupLayout);
    layout->addLayout(cancelSaveLayout);


    ///                                  CONNECTIONS

    /// when the plus button is clicked we display the groupBox
    connect(plusButton, SIGNAL(clicked(bool)), this, SLOT(showGroupLayout()));

    connect(saveBtn, SIGNAL(clicked()), this, SLOT(saveEditSelecPointBtnEvent()));
    connect(nameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkPointName()));

    qDebug() << groupBox->currentIndex();
    connect(cancelBtn, SIGNAL(clicked(bool)), this, SLOT(hideGroupLayout()));

    hide();
    setMaximumWidth(_parent->width()*4/10);
    setMinimumWidth(_parent->width()*4/10);
    layout->setAlignment(Qt::AlignTop);
    layout->setContentsMargins(0,0,0,0);

}

void EditSelectedPointWidget::setSelectedPoint(PointView * const &_pointView, const bool isTemporary){
    _isTemporary = isTemporary;
    pointView = _pointView;
    nameEdit->setText(pointView->getPoint()->getName());
    posXLabel->setText("X : " + QString::number(pointView->getPoint()->getPosition().getX(), 'f', 1));
    posYLabel->setText("Y : " + QString::number(pointView->getPoint()->getPosition().getY(), 'f', 1));
}

void EditSelectedPointWidget::saveEditSelecPointBtnEvent(){
    qDebug() << "saveEditSelecPointBtnEvent called";
    emit pointSaved(groupBox->currentIndex(), posXLabel->text().right(posXLabel->text().length()-4).toDouble(), posYLabel->text().right(posYLabel->text().length()-4).toDouble(), nameEdit->text());
}

void EditSelectedPointWidget::checkPointName(void) const {
    qDebug() << "checkPointName called" << nameEdit->text();
    if(!nameEdit->text().compare("")){
        /// cannot add a point with no name
        saveBtn->setToolTip("The name of your point cannot be empty");
        saveBtn->setEnabled(false);
        return;
    }
    for(int i = 0; i < points->getPoints()->count(); i++){
        std::shared_ptr<Group> group = points->getPoints()->getGroups().at(i);
        for(int j = 0; j < group->count(); j++){
            if(!nameEdit->text().compare(group->getPoints().at(j)->getName(), Qt::CaseInsensitive)){
                qDebug() << nameEdit->text() << " already exists";
                saveBtn->setEnabled(false);
                /// to explain the user why he cannot add its point as it is
                saveBtn->setToolTip("A point with this name already exists, please choose another name for your point");
                return;
            }
        }
    }
    saveBtn->setToolTip("");
    saveBtn->setEnabled(true);
}

void EditSelectedPointWidget::showGroupLayout(void) const {
    /// we disable so that two points cannot be named tmpPoint
    saveBtn->setEnabled(false);
    groupLabel->show();
    groupBox->show();
    saveBtn->show();
    cancelBtn->show();
    plusButton->setEnabled(false);
    plusButton->setToolTip("");
    nameEdit->setReadOnly(false);
    nameEdit->setAutoFillBackground(false);
    nameEdit->setFrame(true);
    nameEdit->setFocusPolicy(Qt::FocusPolicy::StrongFocus);
}

void EditSelectedPointWidget::hideGroupLayout(void) const {
    /// resets the name to tmpPoint if we cancel the creation of the point
    nameEdit->setText(pointView->getPoint()->getName());
    /// hides everything that's related to creating a point
    groupLabel->hide();
    groupBox->hide();
    saveBtn->hide();
    cancelBtn->hide();
    plusButton->setEnabled(true);
    plusButton->setToolTip("Click this button if you want to save this point permanently");
    nameEdit->setReadOnly(true);
    nameEdit->setAutoFillBackground(true);
    nameEdit->setFrame(false);
    nameEdit->setFocusPolicy(Qt::FocusPolicy::NoFocus);
}

void EditSelectedPointWidget::updateGroupBox(const Points& _points){
    groupBox->clear();
    /// we place the default group first
    groupBox->insertItem(0, _points.getDefaultGroup()->getName());
    for(int i = 0; i < _points.count()-1; i++){
        groupBox->insertItem(i+1, _points.getGroups().at(i)->getName());
    }
    /// to set the default group as default
    groupBox->setCurrentIndex(0);
    groupBox->setItemIcon(0, QIcon(":/icons/tick.png"));
}

void EditSelectedPointWidget::keyPressEvent(QKeyEvent* event){
    /// this is the enter key
    if(!event->text().compare("\r")){
        emit pointSaved(groupBox->currentIndex(), posXLabel->text().right(posXLabel->text().length()-4).toDouble(), posYLabel->text().right(posYLabel->text().length()-4).toDouble(), nameEdit->text());
        qDebug() << "enter pressed";
    }
}
