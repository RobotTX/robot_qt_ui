import QtQuick 2.7
import QtQuick.Controls 2.1
import "../Custom"
import "../../Helper/style.js" as Style
import "../../Model/Version"
import "../../View/Settings"

Dialog{

 id: versionDialog

 property Version version
 property string versMessage
 property string feature
 property string langue


 width: 400


 parent: ApplicationWindow.overlay
 header: Label {
     id: customHeader
     background: Rectangle {
         color: "transparent"

     }
     anchors {
         top: parent.top
         left: parent.left
         topMargin: 10
         leftMargin: 10
     }

     font.bold: true
     font.pointSize: 12
     text:{
         if(countIndex == 1)
         {
         qsTr("What's new in Version 1.05")
         }
         else if (countIndex ==2){
         qsTr("What's new in Version 1.04")
         }
         else if (countIndex == 3){
         qsTr("What's new in Version 1.03")
         }
         else if (countIndex == 4){
         qsTr("What's new in Version 1.01")
         }
         else{
         qsTr("What's new in Version 1.0")
         }
     }
 }
 contentItem: Rectangle {

     anchors.fill: parent
      color: "transparent"


     Label {
         id: label
         anchors {
             top: parent.top
             left: parent.left
             right: parent.right
             topMargin: customHeader.height + 20
             leftMargin: 10
             rightMargin: 10
         }
         text: qsTr(versMessage)
         color: Style.midGrey2
         font.pointSize: 10
         wrapMode: Text.WordWrap
     }


}

 Button {
     id: nextbutton

     visible: !(countIndex == 5)


     background: Rectangle {
         radius: 3
         color: Style.darkSkyBlue
         border.width: 1
         border.color: Style.darkSkyBlueBorder

     }
     height: 23
     width: 60
     y : 350
     x : 320
     contentItem: Text{
         text:"NEXT"
         color:"white"
         horizontalAlignment: Text.AlignHCenter
         verticalAlignment: Text.AlignVCenter
     }

     onClicked: {
        countIndex = countIndex + 1;
         if(countIndex == 1){
             versionID.open()
         }
         else if(countIndex == 2){
             versionIDNext.open()
         }
         else if(countIndex == 3){
             versionIDNext1.open()
         }
         else if(countIndex == 4){
             versionIDNext2.open()
         }
         else if(countIndex == 5){
             versionIDNext3.open()
         }
     }
 }
 Button {
     id: backbutton

     visible: !(countIndex == 1)

     background: Rectangle {
         radius: 3
         color: Style.darkSkyBlue
         border.width: 1
         border.color: Style.darkSkyBlueBorder

     }

     anchors {
         left: parent.left
         rightMargin: 10
         leftMargin: 20
         verticalCenter: nextbutton.verticalCenter
     }

     height: 23
     width: 60
     contentItem: Text{
         text:"BACK"
         color:"white"
         horizontalAlignment: Text.AlignHCenter
         verticalAlignment: Text.AlignVCenter
     }

     onClicked: {
         countIndex = countIndex - 1;
          console.log("CountIndex back = " + countIndex)
         if(countIndex == 1){
             versionIDNext.close()
         }
         else if(countIndex == 2){
             versionIDNext1.close()
         }
         else if(countIndex == 3){
             versionIDNext2.close()
         }
         else if(countIndex == 4){
             versionIDNext3.close()
         }
     }
 }
}
