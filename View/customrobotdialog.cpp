#include "customrobotdialog.h"
#include <QLabel>
#include <QLineEdit>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include "View/stylesettings.h"

CustomRobotDialog::CustomRobotDialog(QWidget *parent): QDialog(parent)
{
    grid = new QVBoxLayout(this);
    buttonLayout = new QHBoxLayout();

    QLabel* iconLabel = new QLabel(this);
    iconLabel->setPixmap(QPixmap(":/icons/setting.png").scaled(s_icon_size));
    grid->addWidget(iconLabel);
    iconLabel->show();

    form = new QFormLayout();
    QLabel* nameLabel = new QLabel("Name :", this);
    nameEdit = new QLineEdit(this);
    form->addRow(nameLabel, nameEdit);

    QLabel* SSIDLabel = new QLabel("Wifi SSID : ", this);
    ssidEdit = new QLineEdit(this);
    form->addRow(SSIDLabel, ssidEdit);

    QLabel* passwordLabel = new QLabel("Wifi Password :", this);
    passwordEdit = new QLineEdit(this);
    form->addRow(passwordLabel, passwordEdit);

    cancelButton = new QPushButton("Cancel", this);
    saveButton = new QPushButton("Save", this);

    buttonLayout->addWidget(cancelButton);
    buttonLayout->addWidget(saveButton);

    grid->addLayout(form);

    grid->addLayout(buttonLayout);

}

