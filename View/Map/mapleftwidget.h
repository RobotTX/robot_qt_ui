#ifndef MAPLEFTWIDGET_H
#define MAPLEFTWIDGET_H

class QVBoxLayout;
class QLabel;
class QVBoxLayout;
class MainWindow;
class CustomPushButton;

#include <QWidget>

/**
 * @brief The MapLeftWidget class
 * Class which display the map menu on which user can save/load a map
 */
class MapLeftWidget: public QWidget{
public:
    MapLeftWidget(QWidget* parent, const MainWindow* mainWindow);

    CustomPushButton* getSaveBtn(void) const { return saveBtn; }
    CustomPushButton* getSaveStateBtn(void) const { return saveStateBtn; }
    CustomPushButton* getRestoreStateBtn(void) const { return restoreStateBtn; }

private:
    QVBoxLayout* layout;
    CustomPushButton* saveBtn;
    CustomPushButton* saveStateBtn;
    CustomPushButton* restoreStateBtn;
};

#endif // MAPLEFTWIDGET_H
