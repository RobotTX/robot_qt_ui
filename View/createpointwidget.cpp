#include "View/createpointwidget.h"
#include "View/pointview.h"
#include "Model/point.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QMainWindow>
#include <QLineEdit>
#include <QDebug>
#include <QComboBox>
#include "View/spacewidget.h"
#include <QKeyEvent>
#include "topleftmenu.h"
#include "View/buttonmenu.h"
#include "toplayout.h"



CreatePointWidget::CreatePointWidget(QMainWindow* _parent, std::shared_ptr<Points> _points): QWidget(_parent), parent(_parent), points(_points){
    layout = new QVBoxLayout(this);

    actionButtons = new TopLeftMenu(this);
    actionButtons->disableAll();

    layout->addWidget(actionButtons);
    layout->addWidget(new QLabel("CreatePointWidget", this));

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
    QMapIterator<QString, std::shared_ptr<QVector<std::shared_ptr<PointView>>>> i(*(points->getGroups()));
    int j = 0;
    while (i.hasNext()) {
        i.next();
        if(i.key().compare(TMP_GROUP_NAME) != 0 && i.key().compare(PATH_GROUP_NAME) != 0)
            groupBox->insertItem(points->count()-1-j, i.key());
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

void CreatePointWidget::setSelectedPoint(const std::shared_ptr<PointView> &_pointView){
    qDebug() << "setSelectedPoint called";
    pointView = _pointView;
    nameEdit->setText(pointView->getPoint()->getName());
    posXLabel->setText("X : " + QString::number(pointView->getPoint()->getPosition().getX(), 'f', 1));
    posYLabel->setText("Y : " + QString::number(pointView->getPoint()->getPosition().getY(), 'f', 1));

}

/// emits signal when a user clicks save after editing a point
void CreatePointWidget::saveEditSelecPointBtnEvent(){
    qDebug() << "saveEditSelecPointBtnEvent called";
    emit pointSaved(groupBox->currentText(), posXLabel->text().right(posXLabel->text().length()-4).toDouble(), posYLabel->text().right(posYLabel->text().length()-4).toDouble(), nameEdit->text().simplified());
}

/// to check that the name given to a point is valid ( a point with the same name does not already exist, it is not empty and does not contain ';' '{' or '}'
int CreatePointWidget::checkPointName(void){
    qDebug() << "checkPointName called";

    nameEdit->setText(formatName(nameEdit->text()));
    if(nameEdit->text().simplified().contains(QRegularExpression("[;{}]"))){
        saveBtn->setToolTip("The name of your point cannot contain the characters \";\" and }");
        saveBtn->setEnabled(false);
        emit invalidName(TEXT_COLOR_WARNING, Error::ContainsSemicolon);
        return 0;
    }
    if(!nameEdit->text().simplified().compare("")){
        qDebug() << " I am empty ";
        /// cannot add a point with no name
        saveBtn->setToolTip("The name of your point cannot be empty");
        saveBtn->setEnabled(false);
        emit invalidName(TEXT_COLOR_WARNING, Error::EmptyName);
        return 1;
    }

    QMapIterator<QString, std::shared_ptr<QVector<std::shared_ptr<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        /// determines whether or not the current name is a valid one (not already the name of another point or group)
        bool valid(true);
        i.next();
        if(!i.key().compare(nameEdit->text().simplified(), Qt::CaseInsensitive)){
            qDebug() << "This is already the name of a group" ;
            valid = false;
        }
        if(i.value()){
            for(int j = 0; j < i.value()->size(); j++){
                if(i.value()->at(j)->getPoint()->getName().compare(nameEdit->text().simplified(), Qt::CaseInsensitive) == 0){
                    qDebug() << nameEdit->text() << " already exists";
                    valid = false;
                }
            }
        }
        if(!valid){
            saveBtn->setEnabled(false);
            /// to explain the user why he cannot add its point as it is
            saveBtn->setToolTip("A point or group with this name already exists, please choose another name for your point");
            emit invalidName(TEXT_COLOR_WARNING, Error::AlreadyExists);
            return 2;
        }
    }
    saveBtn->setToolTip("");
    saveBtn->setEnabled(true);
    emit invalidName(TEXT_COLOR_INFO, Error::NoError);
    return 3;
}

/// shows the widgets related to the choice of a group and the saving of a point
void CreatePointWidget::showGroupLayout(void) const {
    qDebug() << "showGroupLayout called";
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

/// hides everything that's related to the creation of a point
void CreatePointWidget::hideGroupLayout(void) const {
    qDebug() << "hideGroupLayout called";
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

/// updates the group box when a new group is created
void CreatePointWidget::updateGroupBox(){
    qDebug() << "updateGroupBox called";
    groupBox->clear();
    QMapIterator<QString, std::shared_ptr<QVector<std::shared_ptr<PointView>>>> i(*(points->getGroups()));
    int j = 0;
    while (i.hasNext()) {
        i.next();
        if(i.key().compare(TMP_GROUP_NAME) != 0 && i.key().compare(PATH_GROUP_NAME) != 0)
            groupBox->insertItem(points->count()-1-j, i.key());
    }
    /// to set the default group as default
    groupBox->setCurrentIndex(0);
}

void CreatePointWidget::keyPressEvent(QKeyEvent* event){
    qDebug() << "CreatePointWidget keyPressEvent called";
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
            emit pointSaved(groupBox->currentText(), posXLabel->text().right(posXLabel->text().length()-4).toDouble(), posYLabel->text().right(posYLabel->text().length()-4).toDouble(), nameEdit->text().simplified());
            break;
        default:
            qDebug() << "if you got here it's probably that u forgot to implement the behavior for one or more error codes";
            break;
        }
    }
}

QString CreatePointWidget::formatName(const QString name) const {
    qDebug() << "formatName called";
    return name;
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
