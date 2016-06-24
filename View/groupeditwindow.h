#ifndef GROUPEDITWINDOW_H
#define GROUPEDITWINDOW_H

class PointsLeftWidget;
class QLineEdit;
class QLabel;
class QVBoxLayout;

#include <QPoint>
#include <QWidget>

class GroupEditWindow: public QWidget
{
public:
    GroupEditWindow(QWidget* parent = 0);

    QLabel* getLabel(void) const { return nameLabel; }
    QLineEdit* getEdit(void) const { return nameEdit; }

private:
    QLabel* nameLabel;
    QLineEdit* nameEdit;
    QVBoxLayout* layout;
};

#endif // GROUPEDITWINDOW_H
