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

EditSelectedPointWidget::EditSelectedPointWidget(QMainWindow* _parent, PointsView* _points){
    parent = _parent;
    points = _points;

    separator = new SpaceWidget(SpaceWidget::HORIZONTAL);

    ///                     PLUS MINUS AND EDIT BUTTONS

    topButtonsLayout = new QHBoxLayout();

    plusButton = new QPushButton(QIcon(":/icons/plus.png"),"");
    plusButton->setIconSize(_parent->size()/10);
    plusButton->setToolTip("Click this button if you want to save this point permanently");

    minusButton = new QPushButton(QIcon(":/icons/minus.png"),"");
    minusButton->setIconSize(_parent->size()/10);
    minusButton->setEnabled(false);

    editButton = new QPushButton(QIcon(":/icons/edit.png"),"");
    editButton->setIconSize(_parent->size()/10);
    editButton->setEnabled(false);

    topButtonsLayout->addWidget(plusButton);
    topButtonsLayout->addWidget(minusButton);
    topButtonsLayout->addWidget(editButton);

    ///                    EYE AND MAP BUTTONS

    eyeMapLayout = new QHBoxLayout();

    eyeButton = new QPushButton(QIcon(":/icons/eye.png"), "");
    eyeButton->setIconSize(_parent->size()/10);
    eyeButton->setEnabled(false);

    mapButton = new QPushButton(QIcon(":/icons/map.png"),"");
    mapButton->setIconSize(_parent->size()/10);
    mapButton->setEnabled(false);

    eyeMapLayout->addWidget(eyeButton);
    eyeMapLayout->addWidget(mapButton);

    ///                  add buttons to the main layout

    layout = new QVBoxLayout();
    layout->addLayout(topButtonsLayout);
    layout->addLayout(eyeMapLayout);

    layout->addWidget(separator);

    nameEdit = new QLineEdit(_parent);
    nameEdit->setStyleSheet ("text-align: left");
    layout->addWidget(nameEdit);

    posXLabel = new QLabel("X : ");
    layout->addWidget(posXLabel);

    posYLabel = new QLabel("Y : ");
    layout->addWidget(posYLabel);

    ///                                  ADD GROUP LABEL AND QComboBox

    groupLayout = new QHBoxLayout();

    groupLabel = new QLabel("Group : ");
    groupLabel->hide();
    groupBox = new QComboBox();
    /// to insert the groups in the box

    for(int i = 0; i < points->getPoints().count(); i++){
        groupBox->insertItem(points->getPoints().count()-1-i, points->getPoints().getGroups().at(i)->getName());
    }
    /// to set the default group as default
    groupBox->setCurrentIndex(0);
    groupBox->hide();

    groupLayout->addWidget(groupLabel);
    groupLayout->addWidget(groupBox);

    ///                                   ADD CANCEL AND SAVE BUTTONS

    cancelSaveLayout = new QHBoxLayout();

    saveBtn = new QPushButton("Save");
    cancelBtn = new QPushButton("Cancel");
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

    connect(groupBox, SIGNAL(activated(int)), this, SLOT(print(int)));
    qDebug() << groupBox->currentIndex();
    connect(cancelBtn, SIGNAL(clicked(bool)), this, SLOT(hideGroupLayout()));

    hide();
    setMaximumWidth(_parent->width()*4/10);
    setMinimumWidth(_parent->width()*4/10);
    layout->setAlignment(Qt::AlignTop);
    setLayout(layout);
}

EditSelectedPointWidget::~EditSelectedPointWidget(){
    delete plusButton;
    delete minusButton;
    delete mapButton;
    delete eyeButton;
    delete editButton;
    delete topButtonsLayout;
    delete eyeMapLayout;
    delete cancelSaveLayout;
    delete layout;
    delete pointView;
    delete nameEdit;
    delete posXLabel;
    delete posYLabel;
    delete points;
    delete saveBtn;
    delete cancelBtn;
    delete parent;
    delete groupBox;
    delete groupLayout;
    delete groupLabel;
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
    emit pointSaved(groupBox->currentIndex(), posXLabel->text().right(posXLabel->text().length()-4).toDouble(), posYLabel->text().right(posYLabel->text().length()-4).toDouble(), nameEdit->text());
}

void EditSelectedPointWidget::checkPointName(void) const {
    qDebug() << "checkPointName called";
    for(int i = 0; i < points->getPoints().count(); i++){
        std::shared_ptr<Group> group = points->getPoints().getGroups().at(i);
        for(int j = 0; j < group->count(); j++){
            if(!nameEdit->text().compare(group->getPoints().at(j)->getName())){
                qDebug() << nameEdit->text() << " already exists";
                saveBtn->setEnabled(false);
                /// to explain the user why he cannot add its point as it is
                saveBtn->setToolTip("A point with this name already exists, please choose another name for your point.");
                return;
            }
        }
    }
    saveBtn->setToolTip("");
    saveBtn->setEnabled(true);
}

void EditSelectedPointWidget::print(int id) const {
    qDebug() << "id " << id;
}

void EditSelectedPointWidget::showGroupLayout() const {
    groupLabel->show();
    groupBox->show();
    saveBtn->show();
    cancelBtn->show();
    plusButton->setEnabled(false);
    plusButton->setToolTip("");
}

void EditSelectedPointWidget::hideGroupLayout() const {
    groupLabel->hide();
    groupBox->hide();
    saveBtn->hide();
    cancelBtn->hide();
    plusButton->setEnabled(true);
    plusButton->setToolTip("Click this button if you want to save this point permanently");
}
