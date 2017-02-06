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


private:
    QVBoxLayout* layout;
    CustomPushButton* saveBtn;
};

#endif // MAPLEFTWIDGET_H
