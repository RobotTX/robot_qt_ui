import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../Custom"
import "../../Model/Path"
import "../../Model/Robot"

Frame {
    id: guideMenuFrame
    padding: 0

    property Paths pathModel
    property Robots robotModel
    property string langue

    signal groupSelected(string groupName)
    signal closeGuideMenu()

    background: Rectangle {
        color: Style.backgroundColorItemGuide
        border.color: Style.backgroundColorDarkGuide
        border.width: 1
    }

    /// This frame is displayed when there is no guide group
//    EmptyMenu {
//        /// Only the invisible "No Group" left and it's empty
//        visible: (pathModel.count === 1 && pathModel.get(0).paths.count === 0) || pathModel.count === 0
//        txt: langue == "English" ? "没有任何目标点，请点击 + 按钮或者双击地图，创建目标点。" : "No guide group"
//        imgSrc: "qrc:/icons/big_point"
//    }

    /// for groups
    Component {
        id: delegateGroup
        GuideListItem {
            width: guideMenuFrame.width/4
            height: guideMenuFrame.height/2
            pathModel: guideMenuFrame.pathModel
            robotModel: guideMenuFrame.robotModel
            langue: guideMenuFrame.langue
            onGroupSelected: guideMenuFrame.groupSelected(groupName)
        }
    }

    Flickable {
        id: flickGroup
        ScrollBar.vertical: ScrollBar { }
        contentHeight: contentItem.childrenRect.height
        anchors {
            fill: parent
            topMargin: 10
        }

        Grid {
            property string selectedGroup: langue == "English" ? Helper.noGroup : Helper.noGroupChinese
            property string selectedPath: (pathModel.count > 0) ? pathModel.get(0).paths.count > 0 ? pathModel.get(0).paths.get(0).pathName : "" : ""

            id: columnId
            columns: 4
            rowSpacing: -20
            columnSpacing: 0

            /// button to return to main menu
            Button {
                id: btnGroup
                height: 310
                width: 230

                background: Rectangle {
                    color: "transparent"
//                    border.width: 1
                }

                Image {
                    id: icon
                    source: "qrc:/icons/back_gold_256256" //imgSrc
                    fillMode: Image.Pad // to not stretch the image
                    anchors{
                        left: parent.left
                        bottom: parent.bottom
                        right: parent.right
                        top: parent.top
                    }
                }

                onClicked: {
                    closeGuideMenu();
                }
            }

            /// groups buttons
            Repeater {
                id: repeaterGroup
                model: pathModel
                delegate: delegateGroup
            }
        }
    }
}
