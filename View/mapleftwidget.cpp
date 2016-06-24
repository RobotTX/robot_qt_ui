#include "mapleftwidget.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QMainWindow>
#include <QLabel>

MapLeftWidget::MapLeftWidget(QMainWindow* parent):QWidget(parent){
    layout = new QVBoxLayout(this);

    /// Back button
    QPushButton* backBtn = new QPushButton(QIcon(":/icons/arrowLeft.png"),"Maps", this);
    backBtn->setStyleSheet ("text-align: left");
    backBtn->setIconSize(parent->size()/10);
    layout->addWidget(backBtn);

    /// Save & load buttons
    QPushButton* saveBtn = new QPushButton(QIcon(":/icons/upload.png"),"Save this map", this);
    QPushButton* loadBtn = new QPushButton(QIcon(":/icons/download.png"),"Load a map", this);

    saveBtn->setIconSize(parent->size()/10);
    loadBtn->setIconSize(parent->size()/10);

    saveBtn->setStyleSheet ("text-align: left");
    loadBtn->setStyleSheet ("text-align: left");

    layout->addWidget(saveBtn);
    layout->addWidget(loadBtn);

    QLabel* label = new QLabel("To scan a Map,\nselect a robot\nand click the button\n\"Scan a map\"", this);
    label->setStyleSheet ("text-align: left");
    layout->addWidget(label);

    connect(backBtn, SIGNAL(clicked()), parent, SLOT(backMapBtnEvent()));
    connect(saveBtn, SIGNAL(clicked()), parent, SLOT(saveMapBtnEvent()));
    connect(loadBtn, SIGNAL(clicked()), parent, SLOT(loadMapBtnEvent()));

    hide();
    setMaximumWidth(parent->width()*4/10);
    setMinimumWidth(parent->width()*4/10);
    layout->setAlignment(Qt::AlignTop);
}

MapLeftWidget::~MapLeftWidget(){
    delete layout;
}
