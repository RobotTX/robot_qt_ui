#ifndef CUSTOM_LINE_EDIT_H
#define CUSTOM_LINE_EDIT_H

#include <QLineEdit>
#include <QFocusEvent>

/**
 * @brief The CustomLineEdit class
 * this class provides a customized line edit that emits a particular signal when losing the focus
 */
class CustomLineEdit: public QLineEdit
{
    Q_OBJECT
public:
    CustomLineEdit(QWidget *parent = Q_NULLPTR);
    CustomLineEdit(const QString &text, QWidget *parent = Q_NULLPTR);
    void initialize(void);
    void updateStyle();
    void setText(const QString &text);

protected:
    void mousePressEvent(QMouseEvent *e);
    void focusOutEvent(QFocusEvent* e);
    void hideEvent(QHideEvent* event);
    void showEvent(QShowEvent *event);
    void resizeEvent(QResizeEvent *event);

signals:
    /// emitted when the QLineEdit loses the focus
    void clickSomewhere(QString name);
    void enableGroupEdit(bool);
};

#endif /// CUSTOM_LINE_EDIT_H
