import QtQuick 2.3
import QtQuick.Controls 1.2
import DMonitor 1.0

ApplicationWindow {
    visible: true
    width: 900
    height: 600
    title: qsTr("DMonitor")


    Rectangle {
        anchors.fill: parent
        property var updateRate: 30


        Timer {
            interval: 1000 / parent.updateRate
            running: true
            repeat: true
            onTriggered: parent.updateView()
        }


        Field {
            id: field
            width: 900
            height: 600
        }

        // list view robot

        Robot {
            id: robot
            width: 10
            height: 10
        }

        function updateView() {
            console.log('updateView')
            field.update()
            robot.update()
        }
    }
}
