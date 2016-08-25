#include "mapleftwidget.h"
#include <QVBoxLayout>
#include "Controller/mainwindow.h"
#include <QLabel>
#include "View/custompushbutton.h"
#include "View/stylesettings.h"

MapLeftWidget::MapLeftWidget(QWidget* parent, MainWindow* mainWindow): QWidget(parent){
    layout = new QVBoxLayout(this);

    /// Save & load buttons
    CustomPushButton* saveBtn = new CustomPushButton(QIcon(":/icons/upload.png"),"Save this map", this);
    CustomPushButton* loadBtn = new CustomPushButton(QIcon(":/icons/download.png"),"Load a map", this);

    /// this button allows a user to save a particular state for the map (point in the center of its screen and zoom)
    CustomPushButton* saveStateBtn = new CustomPushButton(QIcon(":/icons/save_map.png"), "Save the state of the map", this);

    saveBtn->setIconSize(normal_icon_size);
    loadBtn->setIconSize(normal_icon_size);
    saveStateBtn->setIconSize(normal_icon_size);

    layout->addWidget(saveBtn);
    layout->addWidget(loadBtn);
    layout->addWidget(saveStateBtn);

    QLabel* label = new QLabel("To scan a Map,\nselect a robot\nand click the button\n\"Scan a map\"", this);
    label->setStyleSheet ("text-align: left");
    layout->addWidget(label);

    connect(saveBtn, SIGNAL(clicked()), mainWindow, SLOT(saveMapBtnEvent()));
    connect(loadBtn, SIGNAL(clicked()), mainWindow, SLOT(loadMapBtnEvent()));
    connect(saveStateBtn, SIGNAL(clicked()), mainWindow, SLOT(saveMapState()));

    hide();
    /*setMaximumWidth(mainWindow->width()*4/10);
    setMinimumWidth(mainWindow->width()*4/10);*/
    layout->setAlignment(Qt::AlignTop);
    //layout->setContentsMargins(10,0,0,0);
}
