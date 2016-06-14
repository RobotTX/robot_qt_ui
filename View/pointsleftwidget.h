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

#include <QWidget>

/**
 * @brief The PointsLeftWidget class
 * The purpose of this class is to display a menu on the left of the application relative to the Point objects of the Model
 */

class PointsLeftWidget: public QWidget{

public:
    PointsLeftWidget(QMainWindow* parent, Points const& points, bool _groupDisplayed = true);
    ~PointsLeftWidget();

    bool getGroupDisplayed(void) const { return groupDisplayed; }
    void setGroupDisplayed(const bool _groupDisplayed) { groupDisplayed = _groupDisplayed; }
    int getIndexLastGroupClicked(void) const { return indexLastGroupClicked; }
    /**
     * @brief setIndexLastGroupClicked
     * @param index
     * the index of the last group is important to determine which points should be removed and displayed
     */
    void setIndexLastGroupClicked(const int index) { indexLastGroupClicked = index; }

    QPushButton* getBackButton(void) const { return backButton; }
    QPushButton* getBackToGroupsBtn(void) const { return backToGroupsButton; }
    QPushButton* getMinusButton(void) const { return minusButton; }
    QPushButton* getMapButton(void) const { return mapButton; }
    QPushButton* getEditButton(void) const { return editButton; }
    QPushButton* getPlusButton(void) const { return plusButton; }
    QPushButton* getEyeButton(void) const { return eyeButton; }

    GroupButtonGroup* getGroupButtonGroup(void) const { return groupButtonGroup; }

    QLabel* getGroupNameLabel(void) const { return groupNameLabel; }
    QLineEdit* getGroupNameEdit(void) const { return groupNameEdit; }

    VerticalScrollArea* getScrollArea(void) const { return scrollArea; }


private:
    QMainWindow* parent;
    QVBoxLayout* layout;
    QHBoxLayout* eyeMapLayout;
    QHBoxLayout* grid;

    GroupButtonGroup* groupButtonGroup;

    QPushButton* backButton;
    QPushButton* backToGroupsButton;
    QPushButton* minusButton;
    QPushButton* mapButton;
    QPushButton* plusButton;
    QPushButton* editButton;
    QPushButton* eyeButton;

    QLabel* groupNameLabel;
    QLineEdit* groupNameEdit;

    VerticalScrollArea* scrollArea;

    GroupEditWindow* groupWindow;

    /// true if the groups are displayed, false if the points are displayed
    /// this way we can implement two different behavior for the same button minus
    bool groupDisplayed;
    int indexLastGroupClicked;
};

#endif // POINTSLEFTWIDGET_H

