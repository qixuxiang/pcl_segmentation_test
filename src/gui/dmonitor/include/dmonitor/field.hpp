#pragma once
#include <QQuickPaintedItem>

// TODO(MWX): field config

namespace dmonitor {
class Field : public QQuickPaintedItem {
    Q_OBJECT
//    Q_PROPERTY(int m_width READ getWidth WRITE setWidth NOTIFY widthChanged)
//    Q_PROPERTY(int m_height READ getHeight WRITE setHeight NOTIFY heightChanged)
public:
    Field(QQuickItem* parent = 0);
    void paint(QPainter *painter) override;
    ~Field();

// setter & getter
//    inline int getWidth() { return m_width; }
//    inline void setWidth(int w) { m_width = w; }

//    inline int getHeight() { return m_height; }
//    inline int setHeight(int h) { m_height = h; }

//signals:
//    void widthChanged();
//    void heightChanged();

//private:
//    int m_width;
//    int m_height;

};
}
