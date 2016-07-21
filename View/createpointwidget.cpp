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
#include "toplayout.h"



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
    nameEdit->setAlignment(Qt::AlignCenter);
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
    connect(this, SIGNAL(invalidName(QString, CreatePointWidget::Error)), _parent, SLOT(setMessageCreationPoint(QString, CreatePointWidget::Error)));

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

int CreatePointWidget::checkPointName(void){
    nameEdit->setText(formatName(nameEdit->text()));
    qDebug() << nameEdit->text();
    if(nameEdit->text().simplified().contains(QRegularExpression("[;{}]"))){
        qDebug() << " I contain a ; or }";
        saveBtn->setToolTip("The name of your point cannot contain the characters \";\" and }");
        saveBtn->setEnabled(false);
        emit invalidName(TEXT_COLOR_WARNING, Error::ContainsSemicolon);
        return 0;
    }
    qDebug() << "checkPointName called" << nameEdit->text();
    if(!nameEdit->text().simplified().compare("")){
        qDebug() << " I am empty ";
        /// cannot add a point with no name
        saveBtn->setToolTip("The name of your point cannot be empty");
        saveBtn->setEnabled(false);
        emit invalidName(TEXT_COLOR_WARNING, Error::EmptyName);
        return 1;
    }
    for(int i = 0; i < points->getPoints()->count(); i++){
        std::shared_ptr<Group> group = points->getPoints()->getGroups().at(i);
        for(int j = 0; j < group->count(); j++){
            if(!nameEdit->text().simplified().compare(group->getPoints().at(j)->getName(), Qt::CaseInsensitive)){
                qDebug() << nameEdit->text() << " already exists";
                saveBtn->setEnabled(false);
                /// to explain the user why he cannot add its point as it is
                saveBtn->setToolTip("A point with this name already exists, please choose another name for your point");
                emit invalidName(TEXT_COLOR_WARNING, Error::AlreadyExists);
                return 2;
            }
        }
    }
    saveBtn->setToolTip("");
    saveBtn->setEnabled(true);
    emit invalidName(TEXT_COLOR_INFO, Error::NoError);
    return 3;
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
}

void CreatePointWidget::keyPressEvent(QKeyEvent* event){
    /// this is the enter key
    if(!event->text().compare("\r")){
        switch(checkPointName()){
        case 0:
            emit invalidName(TEXT_COLOR_DANGER, Error::ContainsSemicolon);
            break;
        case 1:
            emit invalidName(TEXT_COLOR_DANGER, Error::EmptyName);
            break;
        case 2:
            emit invalidName(TEXT_COLOR_DANGER, Error::AlreadyExists);
            break;
        case 3:
            emit pointSaved(groupBox->currentIndex(), posXLabel->text().right(posXLabel->text().length()-4).toDouble(), posYLabel->text().right(posYLabel->text().length()-4).toDouble(), nameEdit->text().simplified());
            break;
        default:
            qDebug() << "if you got here it's probably that u forgot to implement the behavior for one or more error codes";
            break;
        }
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

void CreatePointWidget::showEvent(QShowEvent* event){
    Q_UNUSED(event)
    cancelBtn->hide();
    saveBtn->hide();
    show();
}
