#include "displayselectedpoint.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QDebug>
#include <QKeyEvent>
#include <QRegularExpression>
#include <assert.h>
#include "Helper/helper.h"
#include "Controller/mainwindow.h"
#include "Controller/Points/pointscontroller.h"
#include "Model/Other/xmlparser.h"
#include "Model/Map/map.h"
#include "Model/Robots/robots.h"
#include "Model/Robots/robot.h"
#include "Model/Paths/pathpoint.h"
#include "Model/Points/point.h"
#include "View/Robots/robotview.h"
#include "View/TopLayout/toplayoutwidget.h"
#include "View/Points/pointview.h"
#include "View/Points/displayselectedpointrobots.h"
#include "View/Other/customscrollarea.h"
#include "View/Other/custompushbutton.h"
#include "View/Other/customlabel.h"
#include "View/Other/customlineedit.h"
#include "View/Other/stylesettings.h"
#include "View/Other/spacewidget.h"

DisplaySelectedPoint::DisplaySelectedPoint(MainWindow* mainWindow, QSharedPointer<Robots> const _robots, QSharedPointer<Points> const& _points, QSharedPointer<Map> const& _map, QSharedPointer<PointView> _pointView):
    QWidget(mainWindow), map(_map), pointView(_pointView), points(_points){

    robots = QSharedPointer<Robots>(_robots);
    layout = new QVBoxLayout(this);
    QVBoxLayout* topLayout = new QVBoxLayout();

    actionButtons = new TopLeftMenu(this);
    actionButtons->getPlusButton()->setEnabled(false);
    actionButtons->getMinusButton()->setCheckable(true);
    actionButtons->getEditButton()->setCheckable(true);
    actionButtons->getGoButton()->setEnabled(false);
    actionButtons->getMapButton()->setCheckable(true);

    actionButtons->getMinusButton()->setToolTip("Click here to remove the point");
    actionButtons->getEditButton()->setToolTip("You can click on this button and then choose between clicking on the map or drag the point to change its position");
    actionButtons->getEditButton()->setCheckable(false);

    topLayout->addWidget(actionButtons);

    nameEdit = new CustomLineEdit(this);
    nameEdit->setFocusPolicy(Qt::FocusPolicy::StrongFocus);
    nameEdit->hide();

    nameLabel = new CustomLabel("OKay", this, true);
    topLayout->addWidget(nameLabel);

    topLayout->addWidget(nameEdit);

    posXLabel = new CustomLabel("X : ", this);
    posXLabel->setWordWrap(true);
    topLayout->addWidget(posXLabel);

    posYLabel = new CustomLabel("Y : ", this);
    posYLabel->setWordWrap(true);
    topLayout->addWidget(posYLabel);
    layout->addLayout(topLayout);

    robotsWidget = new DisplaySelectedPointRobots(this);
    topLayout->addWidget(robotsWidget);

    cancelButton = new CustomPushButton("Cancel", this, CustomPushButton::ButtonType::LEFT_MENU, "center");
    cancelButton->hide();

    saveButton = new CustomPushButton("Save", this, CustomPushButton::ButtonType::LEFT_MENU, "center");
    saveButton->hide();

    editLayout = new QHBoxLayout();
    editLayout->addWidget(cancelButton);
    editLayout->addWidget(saveButton);
    layout->addLayout(editLayout);


    /// to check that a point that's being edited does not get a new name that's already used in the database
    connect(nameEdit, SIGNAL(textEdited(QString)), mainWindow->getPointsController(), SLOT(checkDisplayPointName(QString)));
    connect(robotsWidget, SIGNAL(setSelectedRobotFromPoint(QString)), mainWindow, SLOT(setSelectedRobotFromPointSlot(QString)));
    connect(actionButtons->getMinusButton(), SIGNAL(clicked(bool)), mainWindow->getPointsController(), SLOT(removePointFromInformationMenu()));
    connect(actionButtons->getMapButton(), SIGNAL(clicked(bool)), mainWindow->getPointsController(), SLOT(displayPointMapEvent()));
    connect(actionButtons->getEditButton(), SIGNAL(clicked(bool)), mainWindow->getPointsController(), SLOT(editPointButtonEvent()));
    /// to remove the point by pressing the delete key
    connect(this, SIGNAL(removePoint()), mainWindow->getPointsController(), SLOT(removePointFromInformationMenu()));
    /// to cancel edition of a point on hide event
    connect(this, SIGNAL(cancelEditionPoint()), mainWindow->getPointsController(), SLOT(cancelUpdatePoint()));
    /// to cancel the modifications on an edited point
    connect(cancelButton, SIGNAL(clicked(bool)), mainWindow->getPointsController(), SLOT(cancelUpdatePoint()));
    /// to save the modifications on an edited point
    connect(saveButton, SIGNAL(clicked(bool)), mainWindow->getPointsController(), SLOT(updatePoint()));
    /// the purpose of mainWindow connection is just to propagate the signal to the map view through the main window
    connect(this, SIGNAL(nameChanged(QString, QString)), mainWindow->getPointsController(), SLOT(updatePoint()));
    /// to reset the state of everybody when a user click on a random button while he was editing a point
    connect(this, SIGNAL(resetState(GraphicItemState)),  mainWindow, SLOT(setGraphicItemsState(GraphicItemState)));


    editLayout->setContentsMargins(0, 0, 0, 0);
    layout->setContentsMargins(0, 0, 10, 0);
    topLayout->setAlignment(Qt::AlignTop);
    editLayout->setAlignment(Qt::AlignBottom);
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
}

void DisplaySelectedPoint::displayPointInfo(void) {
    qDebug() << "DisplaySelectedPoint::displayPointInfo called";
    if(pointView){
        if(pointView->isVisible())
            actionButtons->getMapButton()->setToolTip("Click to hide this point");
        else
            actionButtons->getMapButton()->setToolTip("Click to display this point");

        posXLabel->setText("X : " + QString::number(pointView->getPoint()->getPosition().getX(), 'f', 1));
        posYLabel->setText("Y : " + QString::number(pointView->getPoint()->getPosition().getY(), 'f', 1));
        nameLabel->setText(pointView->getPoint()->getName());
    }
}

void DisplaySelectedPoint::mousePressEvent(QEvent* /* unused */){
    qDebug() << "mouse pressed";
    nameLabel->show();
    nameEdit->hide();
}

void DisplaySelectedPoint::keyPressEvent(QKeyEvent* event){
    qDebug() << "DisplaySelectedPoint::keyPressEvent called";
    /// this is the enter key
    if(!event->text().compare("\r") && saveButton->isEnabled())
        emit nameChanged(pointView->getPoint()->getName(), nameEdit->text());
    else if(event->key() == Qt::Key_Delete)
        emit removePoint();

}

void DisplaySelectedPoint::resetWidget(){
    qDebug() << "DisplaySelectedPoint::resetWidget called";

    /// we hide the buttons relative to the edit option and make sure the points properties are not longer modifiable
    nameLabel->show();
    nameEdit->hide();
    actionButtons->getEditButton()->setChecked(false);

    cancelButton->hide();
    saveButton->hide();

    actionButtons->getEditButton()->setEnabled(true);
    actionButtons->getEditButton()->setToolTip("You can click this button and then choose between clicking on the map or drag the point to change its position");

    emit resetState(GraphicItemState::NO_STATE);
}

void DisplaySelectedPoint::hideEvent(QHideEvent *event){
    if(saveButton->isVisible())
        emit cancelEditionPoint();
    resetWidget();
    nameLabel->show();
    nameEdit->hide();
    QWidget::hideEvent(event);
}

void DisplaySelectedPoint::setPointView(QSharedPointer<PointView> _pointView, const QString robotName) {
    qDebug() << "DisplaySelectedPoint::setPointView called";
    assert(_pointView);
    pointView = _pointView;
    robotsWidget->setRobotsWidget(pointView, robots, robotName);
    /*
    if(_pointView){
        pointView = _pointView;

        robotsWidget->setRobotsWidget(pointView, robots, robotName);
    } else {
        qDebug() << "Displayselectedpoint::setpointview pointview null pointer";
    }
    */
}

void DisplaySelectedPoint::resizeEvent(QResizeEvent *event){
    QWidget* widget = static_cast<QWidget*>(parent());
    int maxWidth = widget->width() - 10;
    setFixedWidth(maxWidth);
    QWidget::resizeEvent(event);
}

