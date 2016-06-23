#include "spacewidget.h"

#include <QFrame>
#include <QVBoxLayout>

SpaceWidget::SpaceWidget(SpaceOrientation orientation){
    QVBoxLayout* layout = new QVBoxLayout();
    spaceFrame = new QFrame();

    spaceFrame->setStyleSheet("QFrame {color: grey}");

    if(orientation == SpaceOrientation::HORIZONTAL)
        spaceFrame->setFrameShape(QFrame::HLine);
    else
        spaceFrame->setFrameShape(QFrame::VLine);

    layout->addWidget(spaceFrame);

    setLayout(layout);
}

SpaceWidget::SpaceWidget(){
    delete spaceFrame;
}

void SpaceWidget::setColor(QString color){
    spaceFrame->setStyleSheet("QFrame {color: " + color + "}");
}
