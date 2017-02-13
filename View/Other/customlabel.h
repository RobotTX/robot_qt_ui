#ifndef CUSTOMLABEL_H
#define CUSTOMLABEL_H

#include <QLabel>

/**
 * @brief The CustomLabel class
 * class that provides a customed label
 */
class CustomLabel : public QLabel {
public:
    CustomLabel(QWidget *parent = Q_NULLPTR, bool title = false);
    CustomLabel(const QString &text, QWidget *parent = Q_NULLPTR, bool title = false);
    void initialize();

    /**
     * @brief moveLabel
     * Move the label "..." to the correct position in the button
     */
    void moveLabel();

    /**
     * @brief setText
     * @param text
     * Set the text of the button
     */
    void setText(const QString &text);

protected:

    /**
     * @brief resizeEvent
     * @param event
     * on resize event we set the size of the button depending on the size of its parent
     */
    void resizeEvent(QResizeEvent *event);
    void showEvent(QShowEvent* event);

private:
    QLabel* label;
    bool title;

};

#endif // CUSTOMLABEL_H
