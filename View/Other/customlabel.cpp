#include "customlabel.h"
#include <QDebug>
#include "View/Other/stylesettings.h"

CustomLabel::CustomLabel(QWidget *parent, bool _title) : QLabel(parent), title(_title){
    initialize();
}

CustomLabel::CustomLabel(const QString &text, QWidget *parent, bool _title) : QLabel(text, parent), title(_title){
    initialize();
}

void CustomLabel::initialize(){

    /// Set the style of the label "..." which is used when a text is too long for the
    /// button to display "text..." without modifying the text
    label = new QLabel("...  ", this);

    QString style = "";
    if(title){
        QFont tmpFont = font();
        tmpFont.setPointSize(13);
        setFont(tmpFont);
        label->setFont(tmpFont);
        style += "text-align: center;";
    } else
        style += "text-align: left;";

    /// Style of the label
    setStyleSheet(
                "QLabel { " + style + "}");

    label->setAttribute(Qt::WA_TranslucentBackground, false);
    label->setStyleSheet(
                "QLabel {"
                    "color: #222222;"
                    "padding-right: 5px;"
                    "background-color: " + left_menu_background_color + ";"
                "}"
                "QLabel:disabled {"
                    "color: grey;"
                "}");
    label->hide();
    moveLabel();
}

void CustomLabel::showEvent(QShowEvent* event){
    QLabel::showEvent(event);
    moveLabel();
}

void CustomLabel::resizeEvent(QResizeEvent *event){
    QWidget* widget = static_cast<QWidget*>(parent());
    int maxWidth = widget->width();
    setMaximumWidth(maxWidth);
    QLabel::resizeEvent(event);
    moveLabel();
}

void CustomLabel::setText(const QString &str){
    QLabel::setText(str);
    moveLabel();
}

void CustomLabel::moveLabel(){
    if(!text().isEmpty() && !wordWrap()){
        label->setMinimumHeight(height());
        label->setMaximumHeight(height());

        QFontMetrics fm(font());
        QFontMetrics fmLabel(label->font());
        int strWidth = fm.width(text());
        int maxStrWidth = width() - fmLabel.width(label->text()) - 5;

        /// If the text is too long for the button we display "..." at the right position
        if(strWidth >= maxStrWidth){
            QPoint moveTo = QPoint(0, 0);
            QString str = text();
            QString tmpStr = "";

            for(int i = 0; i < str.size(); i++){
                tmpStr += str.at(i);
                if(fm.width(tmpStr) >= maxStrWidth){
                    tmpStr.remove(tmpStr.size()-1, 1);
                    if(!tmpStr.isEmpty() && tmpStr.at(tmpStr.size()-1) == ' ')
                        tmpStr.remove(tmpStr.size()-1, 1);

                    moveTo = QPoint(fm.width(tmpStr), 0);

                    break;
                }
            }

            label->move(moveTo);
            if(toolTip().compare("") == 0)
                setToolTip(text());
            label->show();

            if(title)
                setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

        } else {
            label->hide();

            if(title)
                setAlignment(Qt::AlignCenter | Qt::AlignVCenter);
        }
    }
}
