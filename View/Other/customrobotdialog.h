#ifndef CUSTOMROBOTDIALOG_H
#define CUSTOMROBOTDIALOG_H

#include <QDialog>
#include <QObject>

class QFormLayout;
class CustomLineEdit;
class QLabel;
class QVBoxLayout;
class QPushButton;
class QHBoxLayout;

class CustomRobotDialog: public QDialog
{
public:
    CustomRobotDialog(QWidget* parent = 0);

    QPushButton* getCancelButton(void) const { return cancelButton; }
    QPushButton* getSaveButton(void) const { return saveButton; }
    CustomLineEdit* getNameEdit(void) const { return nameEdit; }
    CustomLineEdit* getPasswordEdit(void) const { return passwordEdit; }
    CustomLineEdit* getSSIDEdit(void) const { return ssidEdit; }


private:
    QFormLayout* form;

    CustomLineEdit* nameEdit;
    CustomLineEdit* passwordEdit;
    CustomLineEdit* ssidEdit;
    QVBoxLayout* grid;

    QPushButton* cancelButton;
    QPushButton* saveButton;

    QHBoxLayout* buttonLayout;

};

#endif /// CUSTOMROBOTDIALOG_H
