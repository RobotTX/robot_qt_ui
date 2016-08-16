#include "displayselectedgroup.h"
#include <View/customscrollarea.h>
#include <View/pointbuttongroup.h>
#include "View/spacewidget.h"
#include <QVBoxLayout>
#include <QDebug>
#include <QIcon>
#include <QPushButton>
#include <QHBoxLayout>
#include <QMainWindow>
#include <QButtonGroup>
#include <QLabel>
#include "customscrollarea.h"
#include "View/buttonmenu.h"
#include "View/pointview.h"

DisplaySelectedGroup::DisplaySelectedGroup(QMainWindow *parent, QSharedPointer<Points> const& _points) : QWidget(parent), points(_points){
    /// to be able to display a lot of groups and points2
    CustomScrollArea* scrollArea = new CustomScrollArea(this);

    layout = new QVBoxLayout(this);

    actionButtons = new TopLeftMenu(this);
    lastCheckedButton = "";

    actionButtons->disableAll();
    actionButtons->getMinusButton()->setCheckable(false);
    actionButtons->getEditButton()->setCheckable(false);
    actionButtons->getMapButton()->setCheckable(false);

    actionButtons->getPlusButton()->setToolTip("To add a point click on the map");
    actionButtons->getMinusButton()->setToolTip("Select a point and click here to remove it");
    actionButtons->getEditButton()->setToolTip("Select a point and click here to modify it");
    actionButtons->getGoButton()->setToolTip("Select a point and click here to access its information");
    actionButtons->getMapButton()->setToolTip("Select a point and click here to display or hide it on the map");

    layout->addWidget(actionButtons);

    name = new QLabel("\nName : ", this);
    name->setStyleSheet("* {  font-weight: bold; text-decoration:underline}");


    QLabel* label_img  = new QLabel(this);

    QPixmap watermark(":/icons/folder.png");

    QPixmap pixmap_img = watermark.scaled(QSize(this->width()/6,this->width()/6),  Qt::KeepAspectRatio);

    label_img->setPixmap(pixmap_img);

    QHBoxLayout *titleLayout = new QHBoxLayout;
    titleLayout->addWidget(label_img);
    titleLayout->addWidget(name);
    titleLayout->setAlignment(Qt::AlignCenter);

    layout->addLayout(titleLayout);

    pointButtonGroup = new PointButtonGroup(points, 0, this);
    connect(pointButtonGroup->getButtonGroup(), SIGNAL(buttonClicked(QAbstractButton*)), this, SLOT(buttonClickedSlot(QAbstractButton*)));
    scrollArea->setWidget(pointButtonGroup);

    layout->addWidget(scrollArea);
    setMaximumWidth(parent->width()*4/10);
    setMinimumWidth(parent->width()*4/10);
    layout->setContentsMargins(0,0,0,0);

    connect(this, SIGNAL(resetPathPointViews()), parent, SLOT(resetPathPointViewsSlot()));
}

void DisplaySelectedGroup::setName(const QString _name){
    name->setText(_name);
    name->setWordWrap(true);
}

void DisplaySelectedGroup::disableButtons(){
    getActionButtons()->getMapButton()->setCheckable(false);
    uncheck();
    /// resets the minus button
    getActionButtons()->getMinusButton()->setEnabled(false);
    getActionButtons()->getMinusButton()->setToolTip("Select a point and click here to remove it");
    /// resets the eye button
    getActionButtons()->getGoButton()->setEnabled(false);
    getActionButtons()->getGoButton()->setToolTip("Select a point and click here to access its information");
    /// resets the map button
    getActionButtons()->getMapButton()->setEnabled(false);
    getActionButtons()->getMapButton()->setToolTip("Select a point and click here to display or hide it on the map");
    /// resets the edit button
    getActionButtons()->getEditButton()->setEnabled(false);
    getActionButtons()->getEditButton()->setToolTip("Select a point and click here to modify it");
    lastCheckedButton = "";
}

void DisplaySelectedGroup::buttonClickedSlot(QAbstractButton* button){
    qDebug() << "DisplaySelectedGroup::buttonClickedSlot called" << button->isChecked() << button->text() << lastCheckedButton;
    points->setPixmapAll(PointView::PixmapType::NORMAL);
    emit resetPathPointViews();
    if(button->text().compare(lastCheckedButton) == 0){
        disableButtons();
    } else {
        /// changes the pointview on the map to show which point is selected
        QSharedPointer<PointView> pv = points->findPointView(button->text());
        pv->setPixmap(PointView::PixmapType::MID);
        /// if the point is also part of the path we change the point view associated
        if(QSharedPointer<PointView> pathPv = points->findPathPointView(pv->getPoint()->getPosition().getX(), pv->getPoint()->getPosition().getY()))
            pathPv->setPixmap(PointView::PixmapType::MID);
        getActionButtons()->getMapButton()->setCheckable(true);
        /// enables the minus button
        getActionButtons()->getMinusButton()->setEnabled(true);
        getActionButtons()->getMinusButton()->setToolTip("Click to remove the selected point");
        /// enables the eye button
        getActionButtons()->getGoButton()->setEnabled(true);
        getActionButtons()->getGoButton()->setToolTip("Click to see the information of the selected point");
        /// enables the map button
        getActionButtons()->getMapButton()->setEnabled(true);
        if(points->findPointView(button->text())){
            if(points->findPointView(button->text())->isVisible()){
                getActionButtons()->getMapButton()->setChecked(true);
                getActionButtons()->getMapButton()->setToolTip("Click to hide the selected point on the map");
            } else {
                getActionButtons()->getMapButton()->setChecked(false);
                getActionButtons()->getMapButton()->setToolTip("Click to display the selected point on the map");
            }
        } else {
            qDebug() << "DisplaySelectedGroup::buttonClickedSlot could not find the pointView :" << button->text();
        }
        /// enables the edit button
        getActionButtons()->getEditButton()->setEnabled(true);
        getActionButtons()->getEditButton()->setToolTip("Click to modify the selected point");
        lastCheckedButton = button->text();
    }
}

void DisplaySelectedGroup::showEvent(QShowEvent* event){
    Q_UNUSED(event)
    getPointButtonGroup()->setGroup(getPointButtonGroup()->getGroupIndex());
    emit resetPathPointViews();
    QWidget::show();
}
