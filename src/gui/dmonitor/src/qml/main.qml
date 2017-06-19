import QtQuick 2.3
import QtQuick.Controls 1.2
import DMonitor 1.0
import "../js/componentCreation.js" as Spawner

ApplicationWindow {
    id: root
    visible: true
    width: drawArea.width + controlArea.width + controlArea.anchors.leftMargin + 10 * 2
    height: drawArea.height + 10 * 2
    title: qsTr("DMonitor")
    onWidthChanged: drawArea.width = width - (controlArea.width + controlArea.anchors.leftMargin + 10 * 2)

    property string udpAddress: '127.0.0.1'


    Rectangle {
        id: drawArea
        x: 10
        y: 10
        width: field.width
        height: field.height
        onWidthChanged: {
            field.width = width
            field.update()
        }

        property var updateRate: 30
        property var robots: []
        property var balls: []
        property alias field: field

        Timer {
            interval: 1000 / parent.updateRate
            running: true
            repeat: true
            onTriggered: drawArea.updateView()
        }

        // create field before robots
        Field {
            id: field
            width: 900; height: 600
        }


        function updateView() {
            controlArea.modelTab.update()
            robots.forEach(function(rbt, index, array) {
                rbt.update();
                rbt.setIsMonitor(controlArea.modelTab.isMonitor)
            })

            balls.forEach(function(ball, index, array){
                ball.update();
                ball.setIsMonitor(controlArea.modelTab.isMonitor);
            })
        }

        Component.onCompleted: {
            robots = Spawner.createRobots();
            balls = Spawner.createBalls();
            field.update();
        }


    }

    Rectangle {
        id: controlArea
        y: 10
        width: 700
        height: drawArea.height
        anchors.left: drawArea.right
        anchors.topMargin: 10
        anchors.leftMargin: 6
        property alias modelTab: modelTab

        TabView {
            id: modelTab
            x: 0
            y: 0
            width: parent.width - 20
            height: parent.height - 10
            property bool isMonitor: currentIndex === 0 ? true : false


            Tab {
                id: monitor
                title: "Monitor"

                Rectangle {
                    anchors.fill: parent

                    GuiImage {
                        field: drawArea.field
                        x: 0
                        y: 0
                        width: 640
                        height: 480
                        robotId: 1

                        Timer { interval: 1000 / 30
                            running: true
                            repeat: true
                            onTriggered: {
                                parent.update()
                            }
                        }

                        Component.onCompleted: init();
                    }

                }

            }

            Tab {
                id: simulator
                title: "Simulator"
            }
        }
    }
}
