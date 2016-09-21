#ifndef CUSTOMSCROLLAREA_H
#define CUSTOMSCROLLAREA_H

class QEvent;

#include <QScrollArea>


/**
 * @brief The CustomScrollArea class
 * Custom QScrollArea which only displays the vertical (or horizontal) scrollBar
 */
class CustomScrollArea : public QScrollArea {
    Q_OBJECT

public:
    explicit CustomScrollArea(QWidget *parent, bool leftMenu = false, bool vertical = true, QScrollBar* childBar = NULL);

    virtual bool eventFilter(QObject *o, QEvent *e);

protected:
    void resizeEvent(QResizeEvent *event);

private:
    bool vertical;
    bool leftMenu;
    QScrollBar* childBar;
};

#endif
