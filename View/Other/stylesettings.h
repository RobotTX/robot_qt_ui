#ifndef STYLE_SETTINGS_H
#define STYLE_SETTINGS_H

#include <QString>
#include <QSize>


const QSize xxs_icon_size = QSize(10, 10);
const QSize xs_icon_size = QSize(12, 12);
const QSize s_icon_size = QSize(17, 17);
const QSize m_icon_size = QSize(25, 25);
const QSize l_icon_size = QSize(30, 30);
const QSize xl_icon_size = QSize(37, 37);
const QSize xxl_icon_size = QSize(45, 45);

const int s_button_height = 20;
const int m_button_height = 30;
const int l_button_height = 40;

const int top_layout_height = 60;
const int top_left_menu_height = 110;

#define TEXT_COLOR_NORMAL "#333333"
#define TEXT_COLOR_INFO "#31708f"
#define TEXT_COLOR_SUCCESS "#3c763d"
#define TEXT_COLOR_WARNING "#8a6d3b"
#define TEXT_COLOR_DANGER "#a94442"

/// color theme, if you change colors you may have to clean and rebuild for it to be effective
/// for "no color" ( the default grey) , put transparent

const QString button_hover_color = "#e4f3fd";
const QString button_checked_color = "#9bccef";
const QString top_layout_color = "#6ea9d7";
const QString left_menu_background_color = "#e0e0e0";
const QString bottom_menu_background_color = "#e0e0e0";
const QString background_map_view = "white";
const QString text_color = "#306893";
const QString menu_button_color = "transparent";

// green theme
/*
const QString button_hover_color="#d4efca";
const QString button_checked_color="#6b965c";
//const QString color_backgroud_main="white";
const QString top_layout_color="#3f8029";
const QString left_menu_background_color= "white";
const QString bottom_menu_background_color= "white";
const QString background_map_view= "#404244";
*/

#endif // COLORS_H

