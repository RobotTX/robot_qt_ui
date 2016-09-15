#ifndef DISPLAYPATHGROUP_H
#define DISPLAYPATHGROUP_H

#include <QObject>
#include <QWidget>

class CustomScrollArea;
class QMainWindow;
class PathButtonGroup;
class QVBoxLayout;
class TopLeftMenu;
class Paths;
class QAbstractButton;
class MainWindow;
class CustomLabel;

/**
 * @brief The DisplayPathGroup class
 * This class provides a widget to display the paths contained in a group of paths and allows a user to perform actions
 * such as remove or edit on those paths as well as creating new paths
 */
class DisplayPathGroup: public QWidget {
    Q_OBJECT

public:
    DisplayPathGroup(QWidget *_parent, MainWindow *_mainWindow, const QSharedPointer<Paths> &_paths);

    PathButtonGroup* getPathButtonGroup(void) const { return pathButtonGroup; }
    TopLeftMenu* getActionButtons(void) const { return actionButtons; }
    QString getLastCheckedButton(void) const { return lastCheckedButton; }
    void setLastCheckedButton(const QString checkedButton) { lastCheckedButton = checkedButton; }
    CustomLabel* getGroupNameLabel(void) const { return groupNameLabel; }

public:
    /**
     * @brief initializeActionButtons
     * sets enability, checkability and tooltips of buttons
     */
    void initializeActionButtons(void);
    /**
     * @brief setPathsGroup
     * @param groupName
     * display the list of paths contained in the group with an eye icon before the name if the path is displayed
     */
    void setPathsGroup(const QString groupName);
    /// to update the icons to show which path is displayed
    /**
     * @brief updateDisplayedPath
     * updates the icons so that the visible path has the "eye" icon
     */
    void updateDisplayedPath(void);


private slots:

    /**
     * @brief resetMapButton
     * checks or unchecks the eye button appropriately when a new path is selected,
     * the button is checked if the path is visible, unchecked otherwise
     */
    void resetMapButton();

public slots:
    /**
     * @brief enableButtons
     * @param button
     * called when a user select a path to allow him to perform different operations on the selected paths
     */
    void enableButtons(QAbstractButton* button);

protected:
    void showEvent(QShowEvent* event);
    void keyPressEvent(QKeyEvent* event);

signals:
    /// to delete a path with the delete key
    void deletePath();

private:
    MainWindow* mainWindow;
    QSharedPointer<Paths> paths;
    CustomScrollArea* scrollArea;
    PathButtonGroup* pathButtonGroup;
    TopLeftMenu* actionButtons;
    QVBoxLayout* layout;
    QString lastCheckedButton;
    CustomLabel* groupNameLabel;
};

#endif // DISPLAYPATHGROUP_H
