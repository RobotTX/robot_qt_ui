#include "mapleftwidget.h"
#include <QVBoxLayout>
#include "Controller/mainwindow.h"
#include <QLabel>
#include "View/custompushbutton.h"
#include "View/stylesettings.h"

MapLeftWidget::MapLeftWidget(QWidget* parent, const MainWindow *mainWindow): QWidget(parent){
    layout = new QVBoxLayout(this);

    /// Save & load buttons
    saveBtn = new CustomPushButton(QIcon(":/icons/upload.png"), "Save this map", this);
    CustomPushButton* loadBtn = new CustomPushButton(QIcon(":/icons/download.png"), "Load a map", this);

    /// this button allows a user to save a particular state for the map (point in the center of its screen and zoom)
    CustomPushButton* saveStateBtn = new CustomPushButton(QIcon(":/icons/save_map.png"), "Save the state of the map", this);
    CustomPushButton* editBtn = new CustomPushButton(QIcon(":/icons/edit.png"), "Edit the map", this);
    editBtn->setToolTip("Edit a map to remove or create some walls");
    CustomPushButton* mergeBtn = new CustomPushButton(QIcon(":/icons/merge.png"), "Merge maps", this);
    mergeBtn->setToolTip("Merge multiple maps into one");

    saveBtn->setIconSize(s_icon_size);
    loadBtn->setIconSize(s_icon_size);
    saveStateBtn->setIconSize(s_icon_size);
    editBtn->setIconSize(s_icon_size);
    mergeBtn->setIconSize(s_icon_size);

    layout->addWidget(saveBtn);
    layout->addWidget(loadBtn);
    layout->addWidget(saveStateBtn);
    layout->addWidget(editBtn);
    layout->addWidget(mergeBtn);

    QLabel* label = new QLabel("To scan a Map,\nselect a robot\nand click the button\n\"Scan a map\"", this);
    label->setStyleSheet ("text-align: left");
    label->setMargin(10);
    layout->addWidget(label);

    connect(saveBtn, SIGNAL(clicked()), mainWindow, SLOT(saveMapBtnEvent()));
    connect(loadBtn, SIGNAL(clicked()), mainWindow, SLOT(loadMapBtnEvent()));
    connect(editBtn, SIGNAL(clicked()), mainWindow, SLOT(editMapSlot()));
    connect(mergeBtn, SIGNAL(clicked()), mainWindow, SLOT(mergeMapSlot()));

    hide();
    layout->setAlignment(Qt::AlignTop);
    layout->setContentsMargins(0, 0, 10, 0);
}
