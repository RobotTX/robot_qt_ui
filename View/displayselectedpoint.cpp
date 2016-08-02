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

DisplaySelectedPoint::DisplaySelectedPoint(QMainWindow *const _parent, std::shared_ptr<Points> const& _points, std::shared_ptr<Map> const& _map, const std::shared_ptr<PointView> &_pointView, const Origin _origin):
    QWidget(_parent), map(_map), pointView(_pointView), parent(_parent), points(_points), origin(_origin)
{
    layout = new QVBoxLayout(this);

    nameLayout = new QHBoxLayout();

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

    nameLayout->addWidget(nameEdit);

    layout->addLayout(nameLayout);

    posXLabel = new QLabel("X : ", this);
    posXLabel->setWordWrap(true);
    layout->addWidget(posXLabel);

    posYLabel = new QLabel("Y : ", this);
    posYLabel->setWordWrap(true);
    layout->addWidget(posYLabel);

    cancelButton = new QPushButton("Cancel", this);
    cancelButton->hide();

    saveButton = new QPushButton("Save", this);
    saveButton->hide();

    editLayout = new QHBoxLayout();
    editLayout->addWidget(cancelButton);
    editLayout->addWidget(saveButton);

    layout->addLayout(editLayout);

    homeWidget = new QWidget(this);
    QVBoxLayout* homeLayout = new QVBoxLayout(homeWidget);

    SpaceWidget* spaceWidget2 = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    homeLayout->addWidget(spaceWidget2);

    QLabel* homeLabel = new QLabel("This point is the home for the robot :", this);
    homeLabel->setWordWrap(true);
    homeLayout->addWidget(homeLabel);

    robotBtn = new QPushButton("", this);
    homeLayout->addWidget(robotBtn);

    homeWidget->hide();

    layout->addWidget(homeWidget);

    /// to check that a point that's being edited does not get a new name that's already used in the database
    connect(nameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkPointName(QString)));

    setMaximumWidth(_parent->width()*4/10);
    setMinimumWidth(_parent->width()*4/10);

    layout->setContentsMargins(0,0,0,0);
    layout->setAlignment(Qt::AlignTop);

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
    } else {
        qDebug() << "DisplaySelectedPoint::displayPointInfo no pointView found with this pointName :" << pointView->getPoint()->getName();
        pointView->getPoint()->setName("");
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

void DisplaySelectedPoint::setOrigin(const Origin _origin){
    /// if we come from the map there is simply no where
    /// to return so we hide the button
    /// the distinction between when we come from the group menu
    /// and when we come from the points menu is made in the pointBtnEvent
    origin = _origin;
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

    if(pointView){
        /// in case the user had dragged the point around the map or clicked it, this resets the coordinates displayed to the original ones, otherwise this has no effect
        /// reset the position
        posXLabel->setText(QString::number(pointView->getPoint()->getPosition().getX()));
        posYLabel->setText(QString::number(pointView->getPoint()->getPosition().getY()));
        pointView->setPos(static_cast<qreal>(pointView->getPoint()->getPosition().getX()), static_cast<qreal>(pointView->getPoint()->getPosition().getY()));
        /// reset its name in the hover on the map
        nameEdit->setText(pointView->getPoint()->getName());
        if(pointView->getPoint()->isHome()){
            homeWidget->show();
        } else
            homeWidget->hide();
    } else {
        qDebug() << "DisplaySelectedPoint::resetWidget no pointView found with this pointName :" << pointView->getPoint()->getName();
        pointView->getPoint()->setName("");
    }
    emit resetState(GraphicItemState::NO_STATE, true);
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

    QMapIterator<QString, std::shared_ptr<QVector<std::shared_ptr<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();
        if(i.value()){
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
    }
    saveButton->setToolTip("");
    saveButton->setEnabled(true);
    emit invalidName(TEXT_COLOR_INFO, CreatePointWidget::Error::NoError);
    return 3;
}

void DisplaySelectedPoint::setPointName(const QString _pointName, const QString robotName) {
    qDebug() << "DisplaySelectedPoint::setPointView called";
    pointView->getPoint()->setName(_pointName);/*
    std::shared_ptr<PointView> pointView = points->findPointView(pointView->getPoint()->getName());
    std::shared_ptr<Point> point = pointView->getPoint();
    if(pointView){
        if(pointView->getPoint()->isHome()){
            homeWidget->show();
            robotBtn->setText(robotName);
        } else {
            homeWidget->hide();
            robotBtn->setText("");
        }
    } else {
        qDebug() << "DisplaySelectedPoint::setPointName no pointView found with this pointName :" << pointView->getPoint()->getName();
        pointView->getPoint()->setName("");
    }*/
}

QString DisplaySelectedPoint::formatName(const QString name) const {
    qDebug() << "DisplaySelectedPoint::formatName called";
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

