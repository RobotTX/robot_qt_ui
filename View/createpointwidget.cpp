#include "View/createpointwidget.h"
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
#include "topleftmenu.h"
#include "View/buttonmenu.h"



CreatePointWidget::CreatePointWidget(QMainWindow* _parent, PointsView* _points): QWidget(_parent), parent(_parent), points(_points)
{
    layout = new QVBoxLayout(this);

    actionButtons = new TopLeftMenu(this);
    actionButtons->disableAll();

    layout->addWidget(actionButtons);

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
    groupBox->setItemIcon(0, QIcon(":/icons/eye.png"));
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
    connect(actionButtons->getPlusButton(), SIGNAL(clicked(bool)), this, SLOT(showGroupLayout()));

    connect(saveBtn, SIGNAL(clicked()), this, SLOT(saveEditSelecPointBtnEvent()));
    connect(nameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkPointName()));

    qDebug() << groupBox->currentIndex();
    connect(cancelBtn, SIGNAL(clicked(bool)), this, SLOT(hideGroupLayout()));

    /// to display appropriate messages when a user attemps to create a point
    connect(this, SIGNAL(invalidName(CreatePointWidget::Error)), _parent, SLOT(setMessageCreationPoint(CreatePointWidget::Error)));

    hide();
    setMaximumWidth(_parent->width()*4/10);
    setMinimumWidth(_parent->width()*4/10);
    layout->setAlignment(Qt::AlignTop);
    layout->setContentsMargins(0,0,0,0);

}

void CreatePointWidget::setSelectedPoint(PointView * const &_pointView, const bool isTemporary){
    _isTemporary = isTemporary;
    pointView = _pointView;
    nameEdit->setText(pointView->getPoint()->getName());
    posXLabel->setText("X : " + QString::number(pointView->getPoint()->getPosition().getX(), 'f', 1));
    posYLabel->setText("Y : " + QString::number(pointView->getPoint()->getPosition().getY(), 'f', 1));
}

void CreatePointWidget::saveEditSelecPointBtnEvent(){
    qDebug() << "saveEditSelecPointBtnEvent called";
    emit pointSaved(groupBox->currentIndex(), posXLabel->text().right(posXLabel->text().length()-4).toDouble(), posYLabel->text().right(posYLabel->text().length()-4).toDouble(), nameEdit->text().simplified());
}

void CreatePointWidget::checkPointName(void){
    nameEdit->setText(formatName(nameEdit->text()));
    qDebug() << nameEdit->text();
    if(nameEdit->text().simplified().contains(QRegularExpression("[;{}]"))){
        qDebug() << " I contain a ; or }";
        saveBtn->setToolTip("The name of your point cannot contain the characters \";\" and }");
        saveBtn->setEnabled(false);
        emit invalidName(Error::ContainsSemicolon);
        return;
    }
    qDebug() << "checkPointName called" << nameEdit->text();
    if(!nameEdit->text().simplified().compare("")){
        qDebug() << " I am empty ";
        /// cannot add a point with no name
        saveBtn->setToolTip("The name of your point cannot be empty");
        saveBtn->setEnabled(false);
        emit invalidName(Error::EmptyName);
        return;
    }
    for(int i = 0; i < points->getPoints()->count(); i++){
        std::shared_ptr<Group> group = points->getPoints()->getGroups().at(i);
        for(int j = 0; j < group->count(); j++){
            if(!nameEdit->text().simplified().compare(group->getPoints().at(j)->getName(), Qt::CaseInsensitive)){
                qDebug() << nameEdit->text() << " already exists";
                saveBtn->setEnabled(false);
                /// to explain the user why he cannot add its point as it is
                saveBtn->setToolTip("A point with this name already exists, please choose another name for your point");
                emit invalidName(Error::AlreadyExists);
                return;
            }
        }
    }
    saveBtn->setToolTip("");
    saveBtn->setEnabled(true);
    emit invalidName(Error::NoError);
}

void CreatePointWidget::showGroupLayout(void) const {
    /// we disable so that two points cannot be named tmpPoint
    saveBtn->setEnabled(false);
    groupLabel->show();
    groupBox->show();
    saveBtn->show();
    cancelBtn->show();
    actionButtons->getPlusButton()->setEnabled(false);
    actionButtons->getPlusButton()->setToolTip("");
    nameEdit->setReadOnly(false);
    nameEdit->setAutoFillBackground(false);
    nameEdit->setFrame(true);
    nameEdit->setFocusPolicy(Qt::FocusPolicy::StrongFocus);
}

void CreatePointWidget::hideGroupLayout(void) const {
    /// resets the name to tmpPoint if we cancel the creation of the point
    nameEdit->setText(pointView->getPoint()->getName());
    /// hides everything that's related to creating a point
    groupLabel->hide();
    groupBox->hide();
    saveBtn->hide();
    cancelBtn->hide();
    actionButtons->getPlusButton()->setEnabled(true);
    actionButtons->getPlusButton()->setToolTip("Click this button if you want to save this point permanently");
    nameEdit->setReadOnly(true);
    nameEdit->setAutoFillBackground(true);
    nameEdit->setFrame(false);
    nameEdit->setFocusPolicy(Qt::FocusPolicy::NoFocus);
}

void CreatePointWidget::updateGroupBox(const Points& _points){
    groupBox->clear();
    /// we place the default group first
    groupBox->insertItem(0, _points.getDefaultGroup()->getName());
    for(int i = 0; i < _points.count()-1; i++){
        groupBox->insertItem(i+1, _points.getGroups().at(i)->getName());
    }
    /// to set the default group as default
    groupBox->setCurrentIndex(0);
    groupBox->setItemIcon(0, QIcon(":/icons/eye.png"));
}

void CreatePointWidget::keyPressEvent(QKeyEvent* event){
    /// this is the enter key
    if(!event->text().compare("\r")){
        emit pointSaved(groupBox->currentIndex(), posXLabel->text().right(posXLabel->text().length()-4).toDouble(), posYLabel->text().right(posYLabel->text().length()-4).toDouble(), nameEdit->text().simplified());
        qDebug() << "enter pressed";
    }
}

QString CreatePointWidget::formatName(const QString name) const {
    QString ret("");
    bool containsSpace(false);
    bool containsNonSpace(false);
    for(int i = 0; i < name.length(); i++){
        if(!name.at(i).isSpace() || (!containsSpace && containsNonSpace)){
            if(name.at(i).isSpace())
                containsSpace = true;
            else {
                containsNonSpace = true;
                containsSpace = false;
            }
            ret += name.at(i);
        }
    }
    return ret;
}
