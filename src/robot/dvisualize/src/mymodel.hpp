#ifndef MYMODEL_H
#define MYMODEL_H
#include <QAbstractListModel>
#include <QPixmap>
#include <QIcon>

class MyModel : public QAbstractListModel
{
    Q_OBJECT
public:
    MyModel(QObject* parent);

    int rowCount(const QModelIndex &parent) const;

    // implement this for insert items in the view
    bool insertRows(int row, int count, const QModelIndex &parent);

    QVariant data(const QModelIndex &index, int role) const;

    void loadImages(QString filename);

    const QPixmap getImage(int row) const;

    const QPixmap getImage() const;

    void setCurrentIndex(int);

    QPoint getPlatAngle(int index);

private:
    QVector<QPixmap> m_pictures;
    QVector<QIcon> m_pics;
    QVector<QString> m_picname;
    int m_currentIndex;
};

#endif // MYMODEL_H
