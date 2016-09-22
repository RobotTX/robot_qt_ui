#include "displayselectedpoint.h"
#include "Model/point.h"
#include "View/pointview.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QLineEdit>
#include <QDebug>
#include <QKeyEvent>
#include "Model/xmlparser.h"
#include "View/spacewidget.h"
#include "Model/map.h"
#include "toplayout.h"
#include "Model/robots.h"
#include "Model/robot.h"
#include "View/robotview.h"
#include "Model/pathpoint.h"
#include "View/displayselectedpointrobots.h"
#include "View/customscrollarea.h"
#include "View/custompushbutton.h"
#include "View/customlabel.h"
#include "View/customlineedit.h"
#include "View/stylesettings.h"

DisplaySelectedPoint::DisplaySelectedPoint(QWidget* _parent, QSharedPointer<Robots> const _robots, QSharedPointer<Points> const& _points, QSharedPointer<Map> const& _map, QSharedPointer<PointView> _pointView):
    QWidget(_parent), map(_map), pointView(_pointView), points(_points){

    robots = QSharedPointer<Robots>(_robots);
    layout = new QVBoxLayout(this);

    actionButtons = new TopLeftMenu(this);
    actionButtons->getPlusButton()->setEnabled(false);
    actionButtons->getMinusButton()->setCheckable(true);
    actionButtons->getEditButton()->setCheckable(true);
    actionButtons->getGoButton()->setEnabled(false);
    actionButtons->getMapButton()->setCheckable(true);

    actionButtons->getMinusButton()->setToolTip("Click here to remove the point");
    actionButtons->getEditButton()->setToolTip("You can click on this button and then choose between clicking on the map or drag the point to change its position");
    actionButtons->getEditButton()->setCheckable(false);

    layout->addWidget(actionButtons);

    nameEdit = new CustomLineEdit(this);
    nameEdit->setReadOnly(true);
    nameEdit->hide();

    nameLabel = new CustomLabel("OKay", this, true);
    layout->addWidget(nameLabel);

    layout->addWidget(nameEdit);

    posXLabel = new CustomLabel("X : ", this);
    posXLabel->setWordWrap(true);
    layout->addWidget(posXLabel);

    posYLabel = new CustomLabel("Y : ", this);
    posYLabel->setWordWrap(true);
    layout->addWidget(posYLabel);

    robotsWidget = new DisplaySelectedPointRobots(this);
    layout->addWidget(robotsWidget);

    cancelButton = new CustomPushButton("Cancel", this, true, CustomPushButton::ButtonType::LEFT_MENU, "center");
    cancelButton->hide();

    saveButton = new CustomPushButton("Save", this, true, CustomPushButton::ButtonType::LEFT_MENU, "center");
    saveButton->hide();

    editLayout = new QHBoxLayout();
    editLayout->addWidget(cancelButton);
    editLayout->addWidget(saveButton);
    layout->addLayout(editLayout);


    /// to check that a point that's being edited does not get a new name that's already used in the database
    connect(nameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkPointName(QString)));

    editLayout->setContentsMargins(0, 0, 0, 0);
    layout->setContentsMargins(0, 0, 10, 0);
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
    //nameEdit->setReadOnly(true);
    nameLabel->show();
    nameEdit->hide();
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
            /// that is the case where the name of the point has not actually changed
            emit nameChanged(pointView->getPoint()->getName(), pointView->getPoint()->getName());
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
    else if(event->key() == Qt::Key_Delete){
        emit removePoint();
    }
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
    resetWidget();
    nameLabel->show();
    nameEdit->hide();
    QWidget::hideEvent(event);
}

int DisplaySelectedPoint::checkPointName(QString name) {
    qDebug() << "DisplaySelectedPoint::checkPointName called";
    nameEdit->setText(formatName(name));

    /// if it keeps the same name we allow the point to be saved
    if(!formatName(name).compare(pointView->getPoint()->getName())){
        saveButton->setToolTip("");
        saveButton->setEnabled(true);
        emit invalidName(TEXT_COLOR_INFO, CreatePointWidget::Error::NoError);
        return 3;
    }

    /// if the name contains semicolons or curly brackets we forbid it
    if(nameEdit->text().simplified().contains(QRegularExpression("[;{}]")) || nameEdit->text().contains("pathpoint", Qt::CaseInsensitive)){
        qDebug() << " I contain a ; or }";
        saveButton->setToolTip("The name of your point cannot contain the characters \";\" and } or the pattern <pathpoint> ");
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
        qDebug() << "Displayselectedpoint::setpointview pointview null pointer";
    }
}

/// removes extra useless spaces like in " a name    with extra   useless spaces  "
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

void DisplaySelectedPoint::resizeEvent(QResizeEvent *event){
    QWidget* widget = static_cast<QWidget*>(parent());
    int maxWidth = widget->width() - 10;
    setFixedWidth(maxWidth);

    QWidget::resizeEvent(event);
}

