#include "robotpositionrecovery.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include "View/spacewidget.h"
#include "custompushbutton.h"
#include "mergemaplistwidget.h"
#include "View/teleopwidget.h"

RobotPositionRecovery::RobotPositionRecovery(QSharedPointer<Robots> _robots, QWidget *parent) : QWidget(parent), robots(_robots) {
    setAttribute(Qt::WA_DeleteOnClose);
    setMouseTracking(true);

    mainLayout = new QHBoxLayout(this);

}

void RobotPositionRecovery::initializeMenu(){
    /// to create a widget that can contain all the left part of the whole widget
    /// and later on be added to the main layout
    QWidget* menuWidget = new QWidget(this);

    /// the layout containing the left part of the whole widget
    /// including the title, button, list of robots and teleoperation widget
    QVBoxLayout* leftLayout = new QVBoxLayout(menuWidget);

    QVBoxLayout* topMenuLayout = new QVBoxLayout();

    /// Title
    QLabel* titleLabel = new QLabel("Recover robots's positions");
    QFont tmpFont = font();
    tmpFont.setPointSize(13);
    setFont(tmpFont);
    titleLabel->setFont(tmpFont);
    titleLabel->setAlignment(Qt::AlignCenter);
    topMenuLayout->addWidget(titleLabel);

    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    topMenuLayout->addWidget(spaceWidget);

    /// Button to start the recovery
    CustomPushButton* startRecoveryButton = new CustomPushButton("Recover the position of a robot", this);
    startRecoveryButton->setToolTip("If your robot is lost you can try to use this feature to recover its position");
    connect(startRecoveryButton, SIGNAL(clicked()), this, SLOT(recoverRobotPosSlot()));
    topMenuLayout->addWidget(startRecoveryButton);

    /// The widget that lists every robot trying to recover its position
    listWidget = new MergeMapListWidget(this);
    connect(listWidget, SIGNAL(dirKeyPressed(int)), this, SLOT(dirKeyEventSlot(int)));
    topMenuLayout->addWidget(listWidget);

    leftLayout->addLayout(topMenuLayout);

    /// Teleoperation widget
    QVBoxLayout* teleopLayout = new QVBoxLayout();
    TeleopWidget* teleopWidget = new TeleopWidget(this);
    //connect(teleopWidget->getBtnGroup(), SIGNAL(buttonClicked(int)), this, SLOT(teleopCmdSlot(int)));
    teleopLayout->addWidget(teleopWidget);
    leftLayout->addLayout(teleopLayout);



}

void RobotPositionRecovery::dirKeyEventSlot(int key){

}

void RobotPositionRecovery::teleopCmdSlot(int){

}
