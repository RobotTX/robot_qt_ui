#include "spacewidget.h"

#include <QFrame>
#include <QVBoxLayout>

SpaceWidget::SpaceWidget(const SpaceOrientation orientation, QWidget* parent) : QWidget(parent){
    QVBoxLayout* layout = new QVBoxLayout(this);
    spaceFrame = new QFrame(this);

    spaceFrame->setStyleSheet("QFrame {color: grey}");

    if(orientation == SpaceOrientation::HORIZONTAL)
        spaceFrame->setFrameShape(QFrame::HLine);
    else
        spaceFrame->setFrameShape(QFrame::VLine);

    layout->addWidget(spaceFrame);
}

SpaceWidget::~SpaceWidget(){
    delete spaceFrame;
}

void SpaceWidget::setColor(const QString color){
    spaceFrame->setStyleSheet("QFrame {color: " + color + "}");
}
