#ifndef MERGEDIALOG_H
#define MERGEDIALOG_H

#include <QDialog>
class CustomPushButton;
class QVBoxLayout;
class QHBoxLayout;
class QGridLayout;
#include <QObject>

class MergeDialog : public QDialog {
    Q_OBJECT
public :
    MergeDialog(QWidget* parent);

private slots:
    void updateNbMapsChecked(bool checked);

private:
    CustomPushButton* mergeButton;
    QVBoxLayout* mainLayout;
    QHBoxLayout* buttonsLayout;
    QGridLayout* layout;
    unsigned nbMapsChecked;
};

#endif // MERGEDIALOG_H
