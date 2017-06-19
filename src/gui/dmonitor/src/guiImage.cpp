#include "dmonitor/guiImage.hpp"
#include "dvision/frame.hpp"
#include <QPainter>

using namespace dtransmit;
namespace dmonitor {

GuiImage::GuiImage(QQuickItem *parent) : BaseObject(parent) {

}

void GuiImage::init() {
    dvision::Frame::initEncoder();
    m_transmitter = new DTransmit("192.168.255.255");

    m_transmitter->addRawRecv(dconstant::network::robotGuiBase + m_robotId , [&](void* buffer, std::size_t size) {
        dvision::Frame f;
        try {
            f.decode(buffer);
            auto img = f.getRGB();
            m_image = QPixmap::fromImage(QImage(img.data, img.cols, img.rows, img.step, QImage::Format_RGB888));

        } catch (std::exception& e) {
            std::cerr << e.what() << std::endl;
        }
    });

    m_transmitter->startService();
}

void GuiImage::drawMyself(QPainter *painter)
{
    painter->setPen(QPen(Qt::black, 10));
    painter->drawPoint(0, 0);
    painter->drawPixmap(0, 0, width(), height(), m_image);
}

} // namespace dmonitor
