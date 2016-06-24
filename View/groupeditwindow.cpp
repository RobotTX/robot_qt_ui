#include "groupeditwindow.h"
#include <QLineEdit>
#include <QLabel>
#include "pointsleftwidget.h"
#include <QDebug>
#include <QVBoxLayout>
#include "View/spacewidget.h"

GroupEditWindow::GroupEditWindow(QWidget *parent): QWidget(parent){
    layout = new QVBoxLayout(this);
    nameLabel = new QLabel("New group's name : ", this);
    nameEdit = new QLineEdit(this);
    //nameLabel->show();
    //nameEdit->show();
    layout->addWidget(nameLabel);
    layout->addWidget(nameEdit);

    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    layout->addWidget(spaceWidget);
}


