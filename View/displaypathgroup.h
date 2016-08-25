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

class DisplayPathGroup: public QWidget {
    Q_OBJECT

public:
    DisplayPathGroup(QMainWindow *_parent, const QSharedPointer<Paths> &_paths);

    PathButtonGroup* getPathButtonGroup(void) const { return pathButtonGroup; }

public:
    void initializeActionButtons(void);

protected:
    void showEvent(QEvent* event);

private:
    QSharedPointer<Paths> paths;
    CustomScrollArea* scrollArea;
    PathButtonGroup* pathButtonGroup;
    TopLeftMenu* actionButtons;
    QVBoxLayout* layout;
};

#endif // DISPLAYPATHGROUP_H
