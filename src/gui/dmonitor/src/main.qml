import QtQuick 2.3
import QtQuick.Controls 1.2
import DMonitor 1.0

ApplicationWindow {
    id: root
    visible: true
    width: 900
    height: 600
    title: qsTr("DMonitor")

    property string udpAddress: '127.0.0.1'


    Rectangle {
        anchors.fill: parent
        color: "black"
        property var updateRate: 30


        Timer {
            interval: 1000 / parent.updateRate
            running: true
            repeat: true
            onTriggered: parent.updateView()
        }


        Field {
            id: field
        }

        // list view robot

        Robot {
            id: robot
            address: root.udpAddress
            robotId: 4
            width: 100
            height:100
        }

        function updateView() {
            field.update()
            robot.update()
        }

        Component.onCompleted: {
            robot.init()
        }
    }
}
