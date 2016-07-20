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
#include "topleftmenu.h"
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

    QPushButton* getSaveButton(void) const { return saveButton; }
    QPushButton* getCancelButton(void) const { return cancelButton; }

    TopLeftMenu* getActionButtons(void) {return actionButtons;}

    GroupButtonGroup* getGroupButtonGroup(void) const { return groupButtonGroup; }

    QLabel* getGroupNameLabel(void) const { return groupNameLabel; }
    QLineEdit* getGroupNameEdit(void) const { return groupNameEdit; }

    VerticalScrollArea* getScrollArea(void) const { return scrollArea; }

    void setCreatingGroup(const bool create) { creatingGroup = create; }
    void setLastCheckedId(const int  id) {lastCheckedId = id;}

private:
    QString formatName(const QString name) const;
public:
    void disableButtons(void);
    void updateGroupButtonGroup(Points const& points);
    void resetWidget(void);

protected:
    void keyPressEvent(QKeyEvent* event);
    void showEvent(QShowEvent *event);

public slots:
     int checkGroupName(QString name);

private slots:
    void enableButtons(int index);
    void cancelCreationGroup();
    void emitNewGroupSignal();
    void modifyGroupAfterClick(QString name);
    void reconnectModifyEdit();

signals:
    void newGroup(QString name);
    void modifiedGroup(QString name);
    void modifiedGroupAfterClick(QString name);
    void enableReturn();
    void messageCreationGroup(QString);
    void messageCreationPoint();

private:

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
    TopLeftMenu* actionButtons;

    /// true if the groups are displayed, false if the points are displayed
    /// this way we can implement two different behavior for the same button minus
    bool groupDisplayed;
    int indexLastGroupClicked;
    std::shared_ptr<Points> points;
    /// to differenciate the behavior of the enter key
    bool creatingGroup;
    int lastCheckedId;
};

#endif // POINTSLEFTWIDGET_H

