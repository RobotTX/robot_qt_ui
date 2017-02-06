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
class CustomPushButton;

/**
 * @brief The DisplayPathGroup class
 * This class provides a widget to display the paths contained in a group of paths and allows a user to perform actions
 * such as remove or edit on those paths as well as creating new paths
 */
class DisplayPathGroup: public QWidget {
    Q_OBJECT

public:
    DisplayPathGroup(MainWindow *_parent, const QSharedPointer<Paths> &_paths);

    PathButtonGroup* getPathButtonGroup(void) const { return pathButtonGroup; }
    TopLeftMenu* getActionButtons(void) const { return actionButtons; }
    QString getLastCheckedButton(void) const { return lastCheckedButton; }
    CustomLabel* getGroupNameLabel(void) const { return groupNameLabel; }

    void setLastCheckedButton(const QString checkedButton) { lastCheckedButton = checkedButton; }

public:
    /**
     * @brief initializeActionButtons
     * sets enability, checkability and tooltips of buttons
     */
    void initializeActionButtons(void);

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
    void resizeEvent(QResizeEvent *event);

signals:
    /// to delete a path with the delete key
    void deletePath();
    void displayEyeIcon(const QString, CustomPushButton*);
    void checkEyeButton(const QString);
    void updateDisplayedPath();
    void setPathsGroup(const QString);

private:
    QSharedPointer<Paths> paths;
    CustomScrollArea* scrollArea;
    PathButtonGroup* pathButtonGroup;
    TopLeftMenu* actionButtons;
    QVBoxLayout* layout;
    QString lastCheckedButton;
    CustomLabel* groupNameLabel;
};

#endif // DISPLAYPATHGROUP_H
