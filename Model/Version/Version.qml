import QtQuick 2.7
import QtQuick.Controls 2.1


ListModel {
        id: versionmod
        property string langue

        ListElement{
            feature:"NEWESTVERSION"
            message:"- Add track object feature (only functional with robot equipped electro-magnet) in POINT\n
- Fix some typo in Chinese translation\n
- Popup warning window if command robot to auto docking without assigning a home in ROBOT\n
- Auto-focus on inputting name text field when creating new path/point/speech\n
- Add update log in SETTING"
            show:true
        }
        ListElement{
            feature:"NEXTVERSION"
            message:"- Support load audio files(.mp3, .wav) when creating path in PATH\n\t
- Change display order of command in ROBOT\n\t
- Increase max delay time from 999 sec to 9999 sec when creating path in PATH\n\t
- Support increase/decrease sound volume in ROBOT"
            show:true
        }
        ListElement{
            feature:"NEXTVERSION1"
            message:"- THIS IS NEXTVERSION1"
            show:true
        }
        ListElement{
            feature:"NEXTVERSION2"
            message:"- THIS IS NEXTVERSION2"
            show:true
        }
        ListElement{
            feature:"NEXTVERSION3"
            message:"- THIS IS NEXTVERSION3"
            show:true
        }
        function getMessage(_feature){
            for(var i = 0; i < count; i++){
                if(get(i).feature === _feature) {
                    console.log("message = " + get(i).message);
                    return get(i).message
                }
            }
        }


}
