#ifndef MAPLEFTWIDGET_H
#define MAPLEFTWIDGET_H

class QVBoxLayout;
class QLabel;
class QVBoxLayout;
class QMainWindow;

#include <QWidget>

/**
 * @brief The MapLeftWidget class
 * Class which display the map menu on which user can save/load a map
 */
class MapLeftWidget: public QWidget{
public:
    MapLeftWidget(QMainWindow* parent);

private:
    QVBoxLayout* layout;
};

#endif // MAPLEFTWIDGET_H
