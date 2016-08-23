#ifndef DISPLAYSELECTEDPATH_H
#define DISPLAYSELECTEDPATH_H

#include <QWidget>
#include <QVBoxLayout>

class DisplaySelectedPath: public QWidget{
    Q_OBJECT
public:
    DisplaySelectedPath(QWidget* parent);
    QVBoxLayout* layout;
};

#endif // DISPLAYSELECTEDPATH_H
