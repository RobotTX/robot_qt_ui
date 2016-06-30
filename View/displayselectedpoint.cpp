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
#include "Model/group.h"

DisplaySelectedPoint::DisplaySelectedPoint(QMainWindow *_parent, std::shared_ptr<Points> const& _points, PointView* _pointView, const Origin _origin): QWidget(_parent), parent(_parent), origin(_origin)
{
    parent = _parent;
    points = _points;
    pointView = _pointView;

    layout = new QVBoxLayout(this);

    nameLayout = new QHBoxLayout();


    //backButton = new QPushButton(QIcon(":/icons/arrowLeft.png"), "Groups", this);
    //backButton->hide();

    //backButton->setIconSize(_parent->size()/10);
    //layout->addWidget(backButton);


    plusButton = new QPushButton(QIcon(":/icons/plus.png"),"", this);
    plusButton->setIconSize(_parent->size()/10);
    plusButton->setEnabled(false);

    minusButton = new QPushButton(QIcon(":/icons/minus.png"),"", this);
    minusButton->setIconSize(_parent->size()/10);
    minusButton->setToolTip("You can click this button to remove the point");
    minusButton->setCheckable(true);

    editButton = new QPushButton(QIcon(":/icons/edit.png"),"", this);
    editButton->setIconSize(_parent->size()/10);
    editButton->setToolTip("You can click on this button and then choose between clicking on the map or drag the point to change its position");
    editButton->setCheckable(true);

    grid = new QHBoxLayout();
    grid->addWidget(plusButton);
    grid->addWidget(minusButton);
    grid->addWidget(editButton);

    eyeButton = new QPushButton(QIcon(":/icons/eye.png"), "", this);
    eyeButton->setIconSize(_parent->size()/10);
    eyeButton->setEnabled(false);

    mapButton = new QPushButton(QIcon(":/icons/map.png"),"", this);
    mapButton->setCheckable(true);
    mapButton->setIconSize(_parent->size()/10);

    eyeMapLayout = new QHBoxLayout();
    eyeMapLayout->addWidget(eyeButton);
    eyeMapLayout->addWidget(mapButton);

    layout->addLayout(grid);
    layout->addLayout(eyeMapLayout);


    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    layout->addWidget(spaceWidget);

    nameEdit = new QLineEdit(this);
    nameEdit->setReadOnly(true);
    nameEdit->setStyleSheet("* { background-color: rgba(255, 0, 0, 0); font-weight: bold; text-decoration:underline}");
    nameEdit->setAutoFillBackground(true);
    nameEdit->setFrame(false);
    nameEdit->setAlignment(Qt::AlignCenter);
   // nameEdit->setStyleSheet("");

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

    /// to check that a point that's being edited does not get a new name that's already used in the database
    connect(nameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkPointName()));
}

void DisplaySelectedPoint::displayPointInfo(void){
    if(pointView->getPoint()->isDisplayed())
        mapButton->setToolTip("Click to hide this point");
    else
        mapButton->setToolTip("Click to display this point");
    posXLabel->setText("X : " + QString::number(pointView->getPoint()->getPosition().getX(), 'f', 1));
    posYLabel->setText("Y : " + QString::number(pointView->getPoint()->getPosition().getY(), 'f', 1));
    nameEdit->setText(pointView->getPoint()->getName());
}

void DisplaySelectedPoint::mousePressEvent(QEvent* event){
    qDebug() << "mouse pressed";
    nameEdit->setReadOnly(true);
}

void DisplaySelectedPoint::keyPressEvent(QKeyEvent* event){
    /// this is the enter key
    if(!event->text().compare("\r")){
        emit nameChanged(pointView->getPoint()->getName(), nameEdit->text());
        qDebug() << "enter pressed";
    }
}

void DisplaySelectedPoint::setOrigin(const Origin _origin){
    origin = _origin;
    /// if we come from the map there is simply no where
    /// to return so we hide the button
    /// the distinction between when we come from the group menu
    /// and when we come from the points menu is made in the pointBtnEvent

    /*
    if(origin == MAP)

 /*   if(origin == MAP)
>>>>>>> bf15946b3a0b45b447000c69a42472e619fce00d
        backButton->hide();
    else
        backButton->show();
        */
}

void DisplaySelectedPoint::resetWidget(){

    /// to change the aspect of the point name
    nameEdit->setAutoFillBackground(true);
    nameEdit->setFrame(false);
    /// we hide the buttons relative to the edit option and make sure the points properties are not longer modifiable
    nameEdit->setReadOnly(true);
    editButton->setChecked(false);
    cancelButton->hide();
    saveButton->hide();
    /// enable the edit button again and hide the tooltip
    editButton->setEnabled(true);
    editButton->setToolTip("You can click on this button and then choose between clicking on the map or drag the point to change its position");

    if(pointView){
        /// in case the user had dragged the point around the map or clicked it, this resets the coordinates displayed to the original ones, otherwise this has no effect
        /// reset the position
        posXLabel->setText(QString::number(pointView->getPoint()->getPosition().getX()));
        posYLabel->setText(QString::number(pointView->getPoint()->getPosition().getY()));
        pointView->setPos(static_cast<qreal>(pointView->getPoint()->getPosition().getX()), static_cast<qreal>(pointView->getPoint()->getPosition().getY()));
        /// reset its name in the hover on the map
        nameEdit->setText(pointView->getPoint()->getName());
    }
    emit resetState(GraphicItemState::NO_STATE, true);
}

void DisplaySelectedPoint::hideEvent(QHideEvent *event){
    resetWidget();
    QWidget::hideEvent(event);
}

void DisplaySelectedPoint::checkPointName() const {
    qDebug() << "checkPointName called";
    for(int i = 0; i < points->count(); i++){
        std::shared_ptr<Group> group = points->getGroups().at(i);
        for(int j = 0; j < group->count(); j++){
            if(!nameEdit->text().compare(group->getPoints().at(j)->getName())){
                qDebug() << nameEdit->text() << " already exists";
                saveButton->setEnabled(false);
                saveButton->setToolTip("A point with this name already exists, please choose another name for your point.");
                return;
            }
        }
    }
    saveButton->setEnabled(true);
}

void DisplaySelectedPoint::updateCoordinates(double x, double y){
    pointView->setPos(x, y);
    posXLabel->setText("X : " + QString::number(x, 'f', 1));
    posYLabel->setText("Y : " + QString::number(y, 'f', 1));
}

