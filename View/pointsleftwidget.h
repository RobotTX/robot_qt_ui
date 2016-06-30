#ifndef POINTSLEFTWIDGET_H
#define POINTSLEFTWIDGET_H

class VerticalScrollArea;
class PointButtonGroup;
class GroupButtonGroup;
class Points;
class QMainWindow;
class QVBoxLayout;
class QPushButton;
class QLabel;
class QLineEdit;
class QHBoxLayout;
class GroupEditWindow;
class Points;

#include <QWidget>
#include <memory>

/**
 * @brief The PointsLeftWidget class
 * The purpose of this class is to display a menu on the left of the application relative to the Point objects of the Model
 */

class PointsLeftWidget: public QWidget{
    Q_OBJECT
public:
    PointsLeftWidget(QMainWindow* parent, std::shared_ptr<Points> const& _points, bool _groupDisplayed = true);

    bool getGroupDisplayed(void) const { return groupDisplayed; }
    void setGroupDisplayed(const bool _groupDisplayed) { groupDisplayed = _groupDisplayed; }
    int getIndexLastGroupClicked(void) const { return indexLastGroupClicked; }
    std::shared_ptr<Points> getPoints(void) const { return points; }
    /**
     * @brief setIndexLastGroupClicked
     * @param index
     * the index of the last group is important to determine which points should be removed and displayed
     */
    void setIndexLastGroupClicked(const int index) { indexLastGroupClicked = index; }

    QPushButton* getMinusButton(void) const { return minusButton; }
    QPushButton* getMapButton(void) const { return mapButton; }
    QPushButton* getEditButton(void) const { return editButton; }
    QPushButton* getPlusButton(void) const { return plusButton; }
    QPushButton* getEyeButton(void) const { return eyeButton; }
    QPushButton* getSaveButton(void) const { return saveButton; }
    QPushButton* getCancelButton(void) const { return cancelButton; }

    GroupButtonGroup* getGroupButtonGroup(void) const { return groupButtonGroup; }

    QLabel* getGroupNameLabel(void) const { return groupNameLabel; }
    QLineEdit* getGroupNameEdit(void) const { return groupNameEdit; }

    VerticalScrollArea* getScrollArea(void) const { return scrollArea; }

public:
    void disableButtons(void);
    void updateGroupButtonGroup(Points const& points);

private slots:
    void enableButtons(int index);

private:
    QMainWindow* parent;
    QVBoxLayout* layout;
    QHBoxLayout* eyeMapLayout;
    QHBoxLayout* grid;
    QHBoxLayout* creationLayout;

    GroupButtonGroup* groupButtonGroup;

    QPushButton* minusButton;
    QPushButton* mapButton;
    QPushButton* plusButton;
    QPushButton* editButton;
    QPushButton* eyeButton;
    QPushButton* saveButton;
    QPushButton* cancelButton;


    QLabel* groupNameLabel;
    QLineEdit* groupNameEdit;

    VerticalScrollArea* scrollArea;

    GroupEditWindow* groupWindow;

    /// true if the groups are displayed, false if the points are displayed
    /// this way we can implement two different behavior for the same button minus
    bool groupDisplayed;
    int indexLastGroupClicked;
    std::shared_ptr<Points> points;
};

#endif // POINTSLEFTWIDGET_H

