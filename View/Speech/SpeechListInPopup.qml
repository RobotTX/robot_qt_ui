import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Speech"
import "../Custom"

Menu {
    id: selectSpeechMenu
    padding: 0
    width: 140
    property Speechs speechModel
    property int menuIndex: -1
    property int currentMenuIndex: -1
    signal speechSelected(string nameSpeech, string tts)

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        radius: 5
    }

    ColumnLayout {
        spacing: 0
        anchors {
            left: parent.left
            right: parent.right
        }

        Repeater {
            model: speechModel
            delegate: PopupMenuItem {
                labelText: groupName
                Layout.preferredHeight: Style.menuItemHeight+1
                Layout.preferredWidth: parent.width
                leftPadding: Style.menuItemLeftPadding

                Image {
                    id: arrow
                    asynchronous: true
                    source: "qrc:/icons/arrow"
                    fillMode: Image.Pad // For not stretching image
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.right: parent.right
                    anchors.rightMargin: 12
                }
//                onHoveredChanged: if(visible && !speechMenu.visible) currentMenuIndex = index /// desktop
                onHoveredChanged: {
                    if (visible) {
                        speechMenu.open();
                        currentMenuIndex = index;
                    }
                }

//                onHoveredChanged: if (visible) speechMenu.open();

                Menu {
                    id: speechMenu
                    padding: 0
                    width: 140
                    x: parent.width
                    visible: (currentMenuIndex === index && menuIndex === 0)

                    background: Rectangle {
                        color: Style.lightGreyBackground
                        border.color: Style.lightGreyBorder
                        radius: 5
                    }

                    ColumnLayout {
                        spacing: 0
                        anchors {
                            left: parent.left
                            right: parent.right
                        }

                        Repeater {
                            model: speechs
                            delegate: PopupMenuItem {
                                leftPadding: Style.menuItemLeftPadding
                                Layout.preferredHeight: Style.menuItemHeight+1
                                Layout.preferredWidth: parent.width
                                labelText: name
                                onTriggered: {
                                    selectSpeechMenu.speechSelected(name, tts);
                                    speechMenu.close();
                                    selectSpeechMenu.close();
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
