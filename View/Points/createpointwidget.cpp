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


CreatePointWidget::CreatePointWidget(MainWindow *mainWindow):
    QWidget(mainWindow){

    QVBoxLayout* layout = new QVBoxLayout(this);
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
    groupBox->hide();

    updateGroupBox(mainWindow->getPointsController()->getPoints());

    topLayout->addWidget(groupLabel);
    topLayout->addWidget(groupBox);
    layout->addLayout(topLayout);

    /// Add Cancel and Save buttons

    QHBoxLayout* cancelSaveLayout = new QHBoxLayout();

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
    connect(nameEdit, SIGNAL(textEdited(QString)), mainWindow->getPointsController(), SLOT(checkPointName(QString)));

    connect(cancelBtn, SIGNAL(clicked(bool)), this, SLOT(hideGroupLayout(bool)));

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
void CreatePointWidget::saveEditSelecPointBtnEvent(void){
    qDebug() << "CreatePointWidget::saveEditSelecPointBtnEvent called";
    emit pointSaved(groupBox->currentText(), posXLabel->text().right(posXLabel->text().length()-4).toDouble(), posYLabel->text().right(posYLabel->text().length()-4).toDouble(), nameEdit->text().simplified());
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
void CreatePointWidget::updateGroupBox(QSharedPointer<Points> points){
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
    if(!event->text().compare("\r") && saveBtn->isEnabled())
        emit pointSaved(groupBox->currentText(), posXLabel->text().right(posXLabel->text().length()-4).toDouble(), posYLabel->text().right(posYLabel->text().length()-4).toDouble(), nameEdit->text().simplified());
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

