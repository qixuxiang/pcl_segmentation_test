#pragma once
#include "dmonitor/baseObject.hpp"
#include "dvision/VisionInfo.h"
#include <QtCore>

namespace dmonitor {

class ViewRange : public BaseObject {
    Q_OBJECT
public:
    ViewRange(QQuickItem* parent = 0);
    void drawMyself(QPainter* painter) override;
    void setVisionInfo(dvision::VisionInfo info);

  private:
    dvision::VisionInfo m_visionInfo;

};

} //namespace dmonitor
