import QtQuick 2.0
import QtQuick.Controls 1.4


Rectangle {
    id: box
    border.width: 3
    border.color: "#EC5f67"
    height: 30
    width: 30 * 1.67
    color: "#1B2B34"
    property alias text: textArea.text

    Text {
        anchors.centerIn: parent
        id: textArea
    }
}
