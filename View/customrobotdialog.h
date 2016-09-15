#ifndef CUSTOMROBOTDIALOG_H
#define CUSTOMROBOTDIALOG_H

#include <QDialog>
#include <QObject>

class QFormLayout;
class QLineEdit;
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
    QLineEdit* getNameEdit(void) const { return nameEdit; }
    QLineEdit* getPasswordEdit(void) const { return passwordEdit; }
    QLineEdit* getSSIDEdit(void) const { return ssidEdit; }

private:
    QFormLayout* form;

    QLineEdit* nameEdit;
    QLineEdit* passwordEdit;
    QLineEdit* ssidEdit;
    QVBoxLayout* grid;

    QPushButton* cancelButton;
    QPushButton* saveButton;

    QHBoxLayout* buttonLayout;

};

#endif // CUSTOMROBOTDIALOG_H
