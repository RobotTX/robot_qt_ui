#include "customrobotdialog.h"
#include <QLabel>
#include "View/customlineedit.h"
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
    iconLabel->setAlignment(Qt::AlignCenter);
    grid->addWidget(iconLabel);
    iconLabel->show();

    form = new QFormLayout();
    QLabel* nameLabel = new QLabel("Name :", this);
    nameEdit = new CustomLineEdit(this);
    form->addRow(nameLabel, nameEdit);

    QLabel* SSIDLabel = new QLabel("Wifi SSID : ", this);
    ssidEdit = new CustomLineEdit(this);
    form->addRow(SSIDLabel, ssidEdit);

    QLabel* passwordLabel = new QLabel("Wifi Password :", this);
    passwordEdit = new CustomLineEdit("......", this);
    passwordEdit->setEchoMode(QLineEdit::Password);
    form->addRow(passwordLabel, passwordEdit);
    //connect(passwordEdit, SIGNAL(clickSomewhere(QString))

    cancelButton = new QPushButton("Cancel", this);
    saveButton = new QPushButton("Save", this);

    buttonLayout->addWidget(cancelButton);
    buttonLayout->addWidget(saveButton);

    grid->addLayout(form);

    grid->addLayout(buttonLayout);

}
