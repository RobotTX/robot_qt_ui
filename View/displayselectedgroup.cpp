#include "displayselectedgroup.h"
#include <View/customscrollarea.h>
#include <View/pointbuttongroup.h>
#include "View/spacewidget.h"
#include <QVBoxLayout>
#include <QDebug>
#include <QIcon>
#include <QHBoxLayout>
#include <QMainWindow>
#include <QButtonGroup>
#include <QLabel>
#include "customscrollarea.h"
#include "View/pointview.h"
#include "View/custompushbutton.h"
#include "View/customlabel.h"
#include <QKeyEvent>

DisplaySelectedGroup::DisplaySelectedGroup(QWidget* parent, QSharedPointer<Points> const& _points) : QWidget(parent), points(_points), lastCheckedButton(""){
    /// to be able to display a lot of groups and points2
    CustomScrollArea* scrollArea = new CustomScrollArea(this, true);

    layout = new QVBoxLayout(this);
    QVBoxLayout* topLayout = new QVBoxLayout(this);

    actionButtons = new TopLeftMenu(this);
    actionButtons->enableAll(false);
    actionButtons->getMinusButton()->setCheckable(false);
    actionButtons->getEditButton()->setCheckable(false);
    actionButtons->getMapButton()->setCheckable(false);

    actionButtons->getPlusButton()->setToolTip("To add a point click on the map");
    actionButtons->getMinusButton()->setToolTip("Select a point and click here to remove it");
    actionButtons->getEditButton()->setToolTip("Select a point and click here to modify it");
    actionButtons->getGoButton()->setToolTip("Select a point and click here to access its information");
    actionButtons->getMapButton()->setToolTip("Select a point and click here to display or hide it on the map");

    topLayout->addWidget(actionButtons);

    name = new CustomLabel("Name : ", this, true);
    topLayout->addWidget(name);
    layout->addLayout(topLayout);

    pointButtonGroup = new PointButtonGroup(points, 0, this);
    scrollArea->setWidget(pointButtonGroup);
    layout->addWidget(scrollArea);

    connect(pointButtonGroup->getButtonGroup(), SIGNAL(buttonClicked(QAbstractButton*)), this, SLOT(buttonClickedSlot(QAbstractButton*)));

    topLayout->setContentsMargins(0, 0, 10, 0);
    layout->setContentsMargins(0, 0, 0, 0);
}

void DisplaySelectedGroup::setName(const QString _name){
    name->setText(_name);
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
        pv->setPixmap(PointView::PixmapType::SELECTED);
        /// if the point is also part of the path we change the point view associated
        if(QSharedPointer<PointView> pathPv = points->findPathPointView(pv->getPoint()->getPosition().getX(), pv->getPoint()->getPosition().getY()))
            pathPv->setPixmap(PointView::PixmapType::SELECTED);
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
    getPointButtonGroup()->setGroup(getPointButtonGroup()->getGroupName());
    emit resetPathPointViews();
    QWidget::show();
}

void DisplaySelectedGroup::resizeEvent(QResizeEvent *event){
    QWidget* widget = static_cast<QWidget*>(parent());
    int maxWidth = widget->width() - 10;
    setFixedWidth(maxWidth);

    QWidget::resizeEvent(event);
}


/// to delete an point with the delete key
void DisplaySelectedGroup::keyPressEvent(QKeyEvent *event){
    if(pointButtonGroup->getButtonGroup()->checkedButton()){
        if(event->key() == Qt::Key_Delete)
            emit removePoint();
    }
}
