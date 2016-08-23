#include "displayselectedpath.h"
#include <QLabel>

DisplaySelectedPath::DisplaySelectedPath(QWidget *parent):QWidget(parent){
    layout = new QVBoxLayout(this);
    layout->addWidget(new QLabel("Dat page", this));
}
