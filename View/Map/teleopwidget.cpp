#include "teleopwidget.h"
#include <QGridLayout>
#include <QPushButton>
#include <QButtonGroup>

TeleopWidget::TeleopWidget(QWidget* parent) : QWidget(parent), btnGroup(new QButtonGroup(this)){
    QGridLayout* layout = new QGridLayout(this);

    QPushButton* upLeftBtn = new QPushButton("u", this);
    QPushButton* upBtn = new QPushButton("i", this);
    QPushButton* upRightBtn = new QPushButton("o", this);
    QPushButton* leftBtn = new QPushButton("j", this);
    QPushButton* middleBtn = new QPushButton("k", this);
    QPushButton* rightBtn = new QPushButton("l", this);
    QPushButton* downLeftBtn = new QPushButton("m", this);
    QPushButton* downBtn = new QPushButton(",", this);
    QPushButton* downRightBtn = new QPushButton(".", this);

    upLeftBtn->setFlat(true);
    upBtn->setFlat(true);
    upRightBtn->setFlat(true);
    leftBtn->setFlat(true);
    middleBtn->setFlat(true);
    rightBtn->setFlat(true);
    downLeftBtn->setFlat(true);
    downBtn->setFlat(true);
    downRightBtn->setFlat(true);

    btnGroup->addButton(upLeftBtn, 0);
    btnGroup->addButton(upBtn, 1);
    btnGroup->addButton(upRightBtn, 2);
    btnGroup->addButton(leftBtn, 3);
    btnGroup->addButton(middleBtn, 4);
    btnGroup->addButton(rightBtn, 5);
    btnGroup->addButton(downLeftBtn, 6);
    btnGroup->addButton(downBtn, 7);
    btnGroup->addButton(downRightBtn, 8);

    layout->addWidget(upLeftBtn, 0, 0);
    layout->addWidget(upBtn, 0, 1);
    layout->addWidget(upRightBtn, 0, 2);
    layout->addWidget(leftBtn, 1, 0);
    layout->addWidget(middleBtn, 1, 1);
    layout->addWidget(rightBtn, 1, 2);
    layout->addWidget(downLeftBtn, 2, 0);
    layout->addWidget(downBtn, 2, 1);
    layout->addWidget(downRightBtn, 2, 2);
}
