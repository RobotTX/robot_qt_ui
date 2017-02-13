#include "View/Points/createpointwidget.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QDebug>
#include <QComboBox>
#include <QKeyEvent>
#include "Controller/mainwindow.h"
#include "Controller/TopLayout/toplayoutcontroller.h"
#include "Model/Points/point.h"
#include "View/Other/spacewidget.h"
#include "View/LeftMenu/topleftmenu.h"
#include "View/TopLayout/toplayoutwidget.h"
#include "View/Other/custompushbutton.h"
#include "View/Other/customlabel.h"
#include "View/Other/customlineedit.h"
#include "View/Other/stylesettings.h"
#include "View/Points/pointview.h"


CreatePointWidget::CreatePointWidget(MainWindow *mainWindow, QSharedPointer<Points> _points):
    QWidget(mainWindow), points(_points){

    layout = new QVBoxLayout(this);
    QVBoxLayout* topLayout = new QVBoxLayout();

    actionButtons = new TopLeftMenu(this);
    actionButtons->enableAll(false);
    topLayout->addWidget(actionButtons);

    /// to explain the user what to do with his temporary point
    messageCreationLabel = new QLabel("Click \"+\" to save", this);
    messageCreationLabel->setWordWrap(true);
    topLayout->addWidget(messageCreationLabel);

    nameEdit = new CustomLineEdit(this);
    nameEdit->setReadOnly(true);
    nameEdit->setFrame(false);
    topLayout->addWidget(nameEdit);
    nameEdit->hide();

    posXLabel = new QLabel("X : ", this);
    topLayout->addWidget(posXLabel);

    posYLabel = new QLabel("Y : ", this);
    topLayout->addWidget(posYLabel);

    /// Add QLabel and QComboBox
    groupLabel = new QLabel("Group : ", this);
    groupLabel->hide();
    groupBox = new QComboBox(this);

    /// to insert the groups in the box
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    int j = 0;
    while (i.hasNext()) {
        i.next();
        if(i.key().compare(TMP_GROUP_NAME) != 0 && i.key().compare(PATH_GROUP_NAME) != 0)
            groupBox->insertItem(points->count()-1-j, i.key());
    }

    /// to set the default group as default
    groupBox->setCurrentIndex(0);
    groupBox->hide();

    topLayout->addWidget(groupLabel);
    topLayout->addWidget(groupBox);
    layout->addLayout(topLayout);

    /// Add Cancel and Save buttons

    cancelSaveLayout = new QHBoxLayout();

    saveBtn = new CustomPushButton("Save", this, CustomPushButton::ButtonType::LEFT_MENU, "center");
    cancelBtn = new CustomPushButton("Cancel", this, CustomPushButton::ButtonType::LEFT_MENU, "center");
    cancelSaveLayout->addWidget(cancelBtn);
    cancelSaveLayout->addWidget(saveBtn);
    saveBtn->hide();
    cancelBtn->hide();

    layout->addLayout(cancelSaveLayout);

    /// when the plus button is clicked we display the groupBox
    connect(actionButtons->getPlusButton(), SIGNAL(clicked(bool)), this, SLOT(showGroupLayout()));

    connect(saveBtn, SIGNAL(clicked()), this, SLOT(saveEditSelecPointBtnEvent()));
    connect(nameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkPointName()));

    connect(cancelBtn, SIGNAL(clicked(bool)), this, SLOT(hideGroupLayout(bool)));

    /// to display appropriate messages when a user attemps to create a point
    connect(this, SIGNAL(invalidName(QString, CreatePointWidget::Error)), mainWindow, SLOT(setMessageCreationPoint(QString, CreatePointWidget::Error)));

    connect(this, SIGNAL(setMessageTop(QString, QString)), mainWindow->getTopLayoutController(), SLOT(setLabel(QString, QString)));

    connect(this, SIGNAL(pointSaved(QString, double, double, QString)), mainWindow, SLOT(pointSavedEvent(QString, double, double, QString)));

    topLayout->setAlignment(Qt::AlignTop);
    cancelSaveLayout->setAlignment(Qt::AlignBottom);
    topLayout->setContentsMargins(0, 0, 0, 0);
    cancelSaveLayout->setContentsMargins(0, 0, 0, 0);
    layout->setContentsMargins(0, 0, 10, 0);
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
    hide();
}

void CreatePointWidget::setSelectedPoint(QSharedPointer<PointView> _pointView){
    qDebug() << "CreatePointWidget setSelectedPoint called";
    pointView = _pointView;

    /// if that's the temporary point we show the label to create a new point otherwise
    /// we simply display the name of the point
    (_pointView->getPoint()->getName().compare("tmpPoint")) ? nameEdit->setText(pointView->getPoint()->getName()) : messageCreationLabel->show();

    posXLabel->setText("X : " + QString::number(pointView->getPoint()->getPosition().getX(), 'f', 1));
    posYLabel->setText("Y : " + QString::number(pointView->getPoint()->getPosition().getY(), 'f', 1));
}

/// emits signal when a user clicks save after editing a point
void CreatePointWidget::saveEditSelecPointBtnEvent(){
    qDebug() << "CreatePointWidget::saveEditSelecPointBtnEvent called";
    emit pointSaved(groupBox->currentText(), posXLabel->text().right(posXLabel->text().length()-4).toDouble(), posYLabel->text().right(posYLabel->text().length()-4).toDouble(), nameEdit->text().simplified());
}

/// to check that the name given to a point is valid ( a point with the same name does not already exist, it is not empty and does not contain ';' '{' or '}'
int CreatePointWidget::checkPointName(void){
    qDebug() << "checkPointName called";

    nameEdit->setText(formatName(nameEdit->text()));
    if(nameEdit->text().simplified().contains(QRegularExpression("[;{}]")) || nameEdit->text().contains("pathpoint", Qt::CaseInsensitive)){
        saveBtn->setToolTip("The name of your point cannot contain the characters \";\" and } or the pattern <pathpoint> ");
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

    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        /// determines whether or not the current name is a valid one (not already the name of another point or group)
        bool valid(true);
        i.next();
        if(!i.key().compare(nameEdit->text().simplified(), Qt::CaseInsensitive)){
            qDebug() << "This is already the name of a group" ;
            valid = false;
        }
        for(int j = 0; j < i.value()->size(); j++){
            if(i.value()->at(j)->getPoint()->getName().compare(nameEdit->text().simplified(), Qt::CaseInsensitive) == 0){
                qDebug() << nameEdit->text() << " already exists";
                valid = false;
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
void CreatePointWidget::showGroupLayout(void) {
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
    nameEdit->setFocus();
    nameEdit->setText("");
    nameEdit->show();
    messageCreationLabel->hide();
    nameEdit->setPlaceholderText("type your name");
    nameEdit->setFrame(true);
    nameEdit->setFocusPolicy(Qt::FocusPolicy::StrongFocus);

    /// so that the main window sets a message to help the user figuring out what to do
    emit setMessageTop(TEXT_COLOR_INFO, "Choose a name for your point by filling up the corresponding field");
}

/// hides everything that's related to the creation of a point
void CreatePointWidget::hideGroupLayout(const bool pointAdded) {
    qDebug() << "hideGroupLayout called, add point" << pointAdded;
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
    nameEdit->setFrame(false);
    nameEdit->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    nameEdit->hide();
    messageCreationLabel->show();
    if(!pointAdded)
        emit setMessageTop(TEXT_COLOR_INFO, "To save this point permanently click the \"+\" button");
}

/// updates the group box when a new group is created
void CreatePointWidget::updateGroupBox(){
    //qDebug() << "updateGroupBox called";
    groupBox->clear();
    int index(0);

    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));

    while (i.hasNext()) {
        i.next();
        if(i.key().compare(TMP_GROUP_NAME) != 0 && i.key().compare(PATH_GROUP_NAME) != 0){
            groupBox->insertItem(index, i.key());
            index++;
        }
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
            Q_UNREACHABLE();
            qDebug() << "if you got here it's probably that u forgot to implement the behavior for one or more error codes";
            break;
        }
    }
}

QString CreatePointWidget::formatName(const QString name) const {
    qDebug() << "formatName called";
    QString ret("");
    QStringList nameStrList = name.split(" ", QString::SkipEmptyParts);
    for(int i = 0; i < nameStrList.size(); i++){
        if(i > 0)
            ret += " ";
        ret += nameStrList.at(i);
    }
    if(name.size() > 0 && name.at(name.size()-1) == ' ')
        ret += " ";
    return ret;
}

void CreatePointWidget::showEvent(QShowEvent* event){
    Q_UNUSED(event)
    cancelBtn->hide();
    saveBtn->hide();
    groupBox->hide();
    nameEdit->hide();
    nameEdit->setText("");
    show();
}

void CreatePointWidget::resizeEvent(QResizeEvent *event){
    QWidget* widget = static_cast<QWidget*>(parent());
    int maxWidth = widget->width() - 10;
    setMaximumWidth(maxWidth);
    QWidget::resizeEvent(event);
}

