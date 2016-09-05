#ifndef POINTSLEFTWIDGET_H
#define POINTSLEFTWIDGET_H

class CustomScrollArea;
class PointButtonGroup;
class GroupButtonGroup;
class Points;
class QMainWindow;
class QVBoxLayout;
class CustomPushButton;
class QLabel;
class CustomLineEdit;
class QHBoxLayout;
class GroupEditWindow;
class Points;
class QAbstractButton;
class CustomLineEdit;
class MainWindow;

#include <QWidget>
#include <QSharedPointer>
#include "topleftmenu.h"
/**
 * @brief The PointsLeftWidget class
 * The purpose of this class is to display a menu on the left of the application relative to the Point objects of the Model
 */

class PointsLeftWidget: public QWidget{
    Q_OBJECT
public:
    PointsLeftWidget(QWidget* parent, MainWindow* const mainWindow, QSharedPointer<Points> const& _points, bool _groupDisplayed = true);

    bool getGroupDisplayed(void) const { return groupDisplayed; }
    void setGroupDisplayed(const bool _groupDisplayed) { groupDisplayed = _groupDisplayed; }
    QSharedPointer<Points> getPoints(void) const { return points; }
    CustomPushButton* getSaveButton(void) const { return saveButton; }
    CustomPushButton* getCancelButton(void) const { return cancelButton; }
    TopLeftMenu* getActionButtons(void) {return actionButtons;}
    GroupButtonGroup* getGroupButtonGroup(void) const { return groupButtonGroup; }
    QLabel* getGroupNameLabel(void) const { return groupNameLabel; }
    CustomLineEdit* getGroupNameEdit(void) const { return groupNameEdit; }
    CustomScrollArea* getScrollArea(void) const { return scrollArea; }
    QString getLastCheckedId() const { return lastCheckedId;}

    void setCreatingGroup(const bool create) { creatingGroup = create; }
    void setLastCheckedId(const QString  id) {lastCheckedId = id;}

public:
    void disableButtons(void);
    void updateGroupButtonGroup();
    void resetWidget(void);

private:
    QString formatName(const QString name) const;

protected:
    void keyPressEvent(QKeyEvent* event);
    void showEvent(QShowEvent *event);
    void resizeEvent(QResizeEvent *event);

public slots:
     int checkGroupName(QString name);

private slots:
    void enableButtons(QString button);
    void enableButtons(QAbstractButton* button);
    void cancelCreationGroup();
    void emitNewGroupSignal();
    void modifyGroupAfterClick(QString name);
    void reconnectModifyEdit();
    void sendMessageEditGroup(int code);

signals:
    void newGroup(QString name);
    void modifiedGroup(QString name);
    void modifiedGroupAfterClick(QString name);
    void enableReturn();
    void messageCreationGroup(QString, QString);
    void messageCreationPoint();
    void resetPathPointViews();

private:

    QVBoxLayout* layout;
    QHBoxLayout* eyeMapLayout;
    QHBoxLayout* grid;
    QHBoxLayout* creationLayout;

    GroupButtonGroup* groupButtonGroup;

    CustomPushButton* saveButton;
    CustomPushButton* cancelButton;


    QLabel* groupNameLabel;
    CustomLineEdit* groupNameEdit;

    CustomScrollArea* scrollArea;

    TopLeftMenu* actionButtons;

    /// true if the groups are displayed, false if the points are displayed
    /// this way we can implement two different behavior for the same button minus
    bool groupDisplayed;
    QSharedPointer<Points> points;
    /// to differenciate the behavior of the enter key
    bool creatingGroup;
    QString lastCheckedId;
};

#endif // POINTSLEFTWIDGET_H

