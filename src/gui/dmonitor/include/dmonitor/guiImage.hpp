#pragma once
#include "dmonitor/baseObject.hpp"
#include "dtransmit/dtransmit.hpp"
#include <QImage>
#include <QPixmap>

namespace dmonitor {

class GuiImage : public BaseObject {
    Q_OBJECT

public:
    GuiImage(QQuickItem* parent = 0);
    void init() override;
    void drawMyself(QPainter *painter) override;

private:
    dtransmit::DTransmit* m_transmitter;
    QPixmap m_image;
};

} // namespace dmonitor
