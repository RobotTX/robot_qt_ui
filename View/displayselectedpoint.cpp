#include "displayselectedpoint.h"
#include "Model/point.h"
#include "View/pointview.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QMainWindow>
#include <QLineEdit>
#include <QDebug>
#include <QKeyEvent>
#include "Model/xmlparser.h"
#include "View/spacewidget.h"
#include "Model/map.h"
#include "View/buttonmenu.h"
#include "toplayout.h"
#include "Model/robots.h"
#include "Model/robot.h"
#include "View/robotview.h"
#include "Model/pathpoint.h"
#include "View/displayselectedpointrobots.h"
#include <QSet>
#include "View/customscrollarea.h"

DisplaySelectedPoint::DisplaySelectedPoint(QMainWindow *const _parent,  QSharedPointer<Robots> const _robots, QSharedPointer<Points> const& _points, QSharedPointer<Map> const& _map, QSharedPointer<PointView> _pointView):
    QWidget(_parent), map(_map), pointView(_pointView), parent(_parent), points(_points){

    robots = QSharedPointer<Robots>(_robots);
    layout = new QVBoxLayout(this);

    CustomScrollArea* scrollArea = new CustomScrollArea(this);

    QVBoxLayout * scrollLayout = new QVBoxLayout(scrollArea);
    QVBoxLayout * infoLayout = new QVBoxLayout();

    actionButtons = new TopLeftMenu(this);
    actionButtons->getPlusButton()->setEnabled(false);
    actionButtons->getMinusButton()->setCheckable(true);
    actionButtons->getEditButton()->setCheckable(true);
    actionButtons->getGoButton()->setEnabled(false);
    actionButtons->getMapButton()->setCheckable(true);

    actionButtons->getMinusButton()->setToolTip("You can click this button to remove the point");
    actionButtons->getEditButton()->setToolTip("You can click on this button and then choose between clicking on the map or drag the point to change its position");

    layout->addWidget(actionButtons);

    nameEdit = new QLineEdit(this);
    nameEdit->setReadOnly(true);
    nameEdit->setStyleSheet("* { background-color: rgba(255, 0, 0, 0); font-weight: bold; text-decoration:underline}");
    nameEdit->setAutoFillBackground(true);
    nameEdit->setFrame(false);
    nameEdit->setAlignment(Qt::AlignCenter);

    infoLayout->addWidget(nameEdit);

    posXLabel = new QLabel("X : ", this);
    posXLabel->setWordWrap(true);
    infoLayout->addWidget(posXLabel);

    posYLabel = new QLabel("Y : ", this);
    posYLabel->setWordWrap(true);
    infoLayout->addWidget(posYLabel);

    scrollLayout->addLayout(infoLayout);

    cancelButton = new QPushButton("Cancel", this);
    cancelButton->hide();

    saveButton = new QPushButton("Save", this);
    saveButton->hide();

    editLayout = new QHBoxLayout();
    editLayout->addWidget(cancelButton);
    editLayout->addWidget(saveButton);

    scrollLayout->addLayout(editLayout);

    robotsWidget = new DisplaySelectedPointRobots(this);
    scrollLayout->addWidget(robotsWidget);

    /// to check that a point that's being edited does not get a new name that's already used in the database
    connect(nameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkPointName(QString)));
    connect(robotsWidget, SIGNAL(setSelectedRobotFromPoint(QString)), parent, SLOT(setSelectedRobotFromPointSlot(QString)));

    setMaximumWidth(_parent->width()*4/10);
    setMinimumWidth(_parent->width()*4/10);

    scrollLayout->setContentsMargins(20,0,0,0);
    scrollLayout->setAlignment(Qt::AlignTop);

    layout->addWidget(scrollArea);


    layout->setContentsMargins(0,0,0,0);

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
        nameEdit->setText(pointView->getPoint()->getName());

        //robotsWidget->setRobotsWidget(pointView, robots);
    }
}

void DisplaySelectedPoint::mousePressEvent(QEvent* /* unused */){
    qDebug() << "mouse pressed";
    nameEdit->setReadOnly(true);
}

void DisplaySelectedPoint::keyPressEvent(QKeyEvent* event){
    qDebug() << "DisplaySelectedPoint::keyPressEvent called";
    /// this is the enter key
    if(!event->text().compare("\r")){
        switch(checkPointName(nameEdit->text())){
        case 0:
            emit invalidName(TEXT_COLOR_DANGER, CreatePointWidget::Error::ContainsSemicolon);
            break;
        case 1:
            emit invalidName(TEXT_COLOR_DANGER, CreatePointWidget::Error::EmptyName);
            break;
        case 2:
            emit invalidName(TEXT_COLOR_DANGER, CreatePointWidget::Error::AlreadyExists);
            break;
        case 3:
            emit nameChanged(pointView->getPoint()->getName(), nameEdit->text());
            break;
        default:
            qDebug() << "if you got here, it's probably that you forgot to define the behavior for one or more error codes";
            break;
        }
    }
}

void DisplaySelectedPoint::resetWidget(){
    qDebug() << "DisplaySelectedPoint::resetWidget called";

    /// to change the aspect of the point name
    nameEdit->setAutoFillBackground(true);
    nameEdit->setFrame(false);
    /// we hide the buttons relative to the edit option and make sure the points properties are not longer modifiable
    nameEdit->setReadOnly(true);
    actionButtons->getEditButton()->setChecked(false);

    cancelButton->hide();
    saveButton->hide();

    actionButtons->getEditButton()->setEnabled(true);
    actionButtons->getEditButton()->setToolTip("You can click on this button and then choose between clicking on the map or drag the point to change its position");

    emit resetState(GraphicItemState::NO_STATE);
}

void DisplaySelectedPoint::hideEvent(QHideEvent *event){
    resetWidget();
    QWidget::hideEvent(event);
}

int DisplaySelectedPoint::checkPointName(QString name) {
    qDebug() << "DisplaySelectedPoint::checkPointName called";
    nameEdit->setText(formatName(name));
    qDebug() << "checking " << nameEdit->text();
    if(nameEdit->text().simplified().contains(QRegularExpression("[;{}]"))){
        qDebug() << " I contain a ; or }";
        saveButton->setToolTip("The name of your point cannot contain the characters \";\" and }");
        saveButton->setEnabled(false);
        emit invalidName(TEXT_COLOR_WARNING, CreatePointWidget::Error::ContainsSemicolon);
        return 0;
    }
    if(!nameEdit->text().simplified().compare("")){
        qDebug() << " I am empty ";
        /// cannot add a point with no name
        saveButton->setToolTip("The name of your point cannot be empty");
        saveButton->setEnabled(false);
        emit invalidName(TEXT_COLOR_WARNING, CreatePointWidget::Error::EmptyName);
        return 1;
    }

    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();
        for(int j = 0; j < i.value()->size(); j++){
            if(i.value()->at(j)->getPoint()->getName().compare(nameEdit->text().simplified(), Qt::CaseInsensitive) == 0){
                qDebug() << nameEdit->text() << " already exists";
                saveButton->setEnabled(false);
                /// to explain the user why he cannot add its point as it is
                saveButton->setToolTip("A point with this name already exists, please choose another name for your point");
                emit invalidName(TEXT_COLOR_WARNING, CreatePointWidget::Error::AlreadyExists);
                return 2;
            }
        }
    }
    saveButton->setToolTip("");
    saveButton->setEnabled(true);
    emit invalidName(TEXT_COLOR_INFO, CreatePointWidget::Error::NoError);
    return 3;
}

void DisplaySelectedPoint::setPointView(QSharedPointer<PointView> _pointView, const QString robotName) {
    qDebug() << "DisplaySelectedPoint::setPointView called";

    if(_pointView){
        pointView = _pointView;

        robotsWidget->setRobotsWidget(pointView, robots, robotName);
    } else {
        qDebug() << "displayselectedpoint::setpointview pointview null pointer";
    }
}

QString DisplaySelectedPoint::formatName(const QString name) const {
    qDebug() << "DisplaySelectedPoint::formatName called";

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

