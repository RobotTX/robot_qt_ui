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

class DisplayPathGroup: public QWidget {
    Q_OBJECT

public:
    DisplayPathGroup(QMainWindow *_parent, const QSharedPointer<Paths> &_paths);

    PathButtonGroup* getPathButtonGroup(void) const { return pathButtonGroup; }
    TopLeftMenu* getActionButtons(void) const { return actionButtons; }

public:
    void initializeActionButtons(void);

private slots:
    void enableButtons(QAbstractButton* button);

protected:
    void showEvent(QShowEvent* event);

private:
    QSharedPointer<Paths> paths;
    CustomScrollArea* scrollArea;
    PathButtonGroup* pathButtonGroup;
    TopLeftMenu* actionButtons;
    QVBoxLayout* layout;
    QString lastCheckedButton;
};

#endif // DISPLAYPATHGROUP_H
