// clickable label

#ifndef CLABEL_H
#define CLABEL_H
#include <QLabel>

const int defaultW = 640;
const int defalutH = 480;

class CLabel : public QLabel
{
    Q_OBJECT
signals:
    void clicked(QMouseEvent* ev);

public:
    CLabel(QWidget* parent);

    void mousePressEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;

    inline int x() const { return mouseX; }
    inline int y() const { return mouseY; }

private:
    int mouseX;
    int mouseY;
};


#endif // CLABEL_H
