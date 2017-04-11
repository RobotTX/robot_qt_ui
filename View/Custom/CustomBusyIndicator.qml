import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

BusyIndicator {
    id: control
    property int nb: 0
    property int size: Math.min(parent.width, parent.height, 70)
    Component.onCompleted: console.log("size : " + size)
    padding: 0

    contentItem: Item {
        width: size
        height: size
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter

        Item {
            id: item
            width: size
            height: size
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
            opacity: control.running ? 1 : 0

            Behavior on opacity {
                OpacityAnimator {
                    duration: 250
                }
            }

            Timer {
                interval: 200
                running: control.running
                repeat: true
                onTriggered: {
                    nb = (nb + 1) % repeater.count;
                    listModel.setProperty(nb, "white", !listModel.get(nb).white);
                }
            }

            ListModel {
                id: listModel
                Component.onCompleted: {
                    for(var i = 0; i < 8; i++)
                        listModel.append({
                            "white": true
                        });
                }
            }

            Repeater {
                id: repeater
                model: listModel

                delegate: Rectangle {
                    x: item.width / 2 - width / 2
                    y: item.height / 2 - height / 2
                    implicitWidth: size / 5
                    implicitHeight: size / 5
                    radius: width / 2
                    color: white ? "white" : Style.darkSkyBlue
                    border.color: Style.darkSkyBlueBorder
                    transform: [
                        Translate {
                            y: -Math.min(item.width, item.height) * 0.5 + width / 2
                        },
                        Rotation {
                            angle: index / repeater.count * 360
                            origin.x: width / 2
                            origin.y: height / 2
                        }
                    ]
                }
            }
            /*Repeater {
                id: repeater
                model: listModel

                delegate: Rectangle {
                    x: item.width / 2 - width / 2
                    y: item.height / 2 - height / 2
                    implicitWidth: 10
                    implicitHeight: 10
                    radius: 5
                    color: white ? "white" : Style.darkSkyBlue
                    border.color: Style.darkSkyBlueBorder
                    transform: [
                        Translate {
                            y: -Math.min(item.width, item.height) * 0.5 + 5
                        },
                        Rotation {
                            angle: index / repeater.count * 360
                            origin.x: 5
                            origin.y: 5
                        }
                    ]
                }
            }*/
        }
    }
}
