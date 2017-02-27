#include "View/Map/mergemaplistwidget.h"
#include "View/Other/stylesettings.h"
#include <QKeyEvent>

MergeMapListWidget::MergeMapListWidget(QWidget *parent) : QListWidget(parent){
    setFrameShape(QFrame::NoFrame);
    viewport()->setAutoFillBackground(false);
    setAttribute(Qt::WA_MacShowFocusRect, false);
    setStyleSheet(" QListWidget {color: red;}\
                  QListWidget::item {border-bottom: 1px solid; border-bottom-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, x3: 0, y3: 0, stop: 0 transparent, stop: 0.5 #949494, stop: 1 transparent); }\
                      QListWidget::item:hover {background-color:" + button_hover_color + ";}\
                      QListWidget::item:selected {background-color:" + button_checked_color + ";}");
}

void MergeMapListWidget::keyPressEvent(QKeyEvent* event){
    /// We want to use the directional keys to move the map on the scene and not to navigate in the list
    emit dirKeyPressed(event->key());
}
