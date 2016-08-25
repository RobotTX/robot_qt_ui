#include "mapleftwidget.h"
#include <QVBoxLayout>
#include <QMainWindow>
#include <QLabel>
#include "View/custompushbutton.h"

MapLeftWidget::MapLeftWidget(QMainWindow* parent): QWidget(parent){
    layout = new QVBoxLayout(this);

    /// Save & load buttons
    CustomPushButton* saveBtn = new CustomPushButton(QIcon(":/icons/upload.png"),"Save this map", this);
    CustomPushButton* loadBtn = new CustomPushButton(QIcon(":/icons/download.png"),"Load a map", this);

    /// this button allows a user to save a particular state for the map (point in the center of its screen and zoom)
    CustomPushButton* saveStateBtn = new CustomPushButton(QIcon(":/icons/save_map.png"), "Save the state of the map", this);

    saveBtn->setIconSize(parent->size()/10);
    loadBtn->setIconSize(parent->size()/10);
    saveStateBtn->setIconSize(parent->size()/10);

    layout->addWidget(saveBtn);
    layout->addWidget(loadBtn);
    layout->addWidget(saveStateBtn);

    QLabel* label = new QLabel("To scan a Map,\nselect a robot\nand click the button\n\"Scan a map\"", this);
    label->setStyleSheet ("text-align: left");
    layout->addWidget(label);

    connect(saveBtn, SIGNAL(clicked()), parent, SLOT(saveMapBtnEvent()));
    connect(loadBtn, SIGNAL(clicked()), parent, SLOT(loadMapBtnEvent()));
    connect(saveStateBtn, SIGNAL(clicked()), parent, SLOT(saveMapState()));

    hide();
    setMaximumWidth(parent->width()*4/10);
    setMinimumWidth(parent->width()*4/10);
    layout->setAlignment(Qt::AlignTop);
    //layout->setContentsMargins(10,0,0,0);
}
