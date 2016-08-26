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
class QLabel;

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
    QLabel* getGroupNameLabel(void) const { return groupNameLabel; }

public:
    void initializeActionButtons(void);
    void setPathsGroup(const QString groupName);

private slots:
    void enableButtons(QAbstractButton* button);
    void resetMapButton();

protected:
    void showEvent(QShowEvent* event);

private:
    MainWindow* mainWindow;
    QSharedPointer<Paths> paths;
    CustomScrollArea* scrollArea;
    PathButtonGroup* pathButtonGroup;
    TopLeftMenu* actionButtons;
    QVBoxLayout* layout;
    QString lastCheckedButton;
    QLabel* groupNameLabel;
};

#endif // DISPLAYPATHGROUP_H
