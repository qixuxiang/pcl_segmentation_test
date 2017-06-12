#ifndef MYMODEL_H
#define MYMODEL_H
#include <QAbstractListModel>
#include <QtCore>
#include <QPixmap>
#include <QIcon>

class MyModel : public QAbstractListModel
{
    Q_OBJECT
public:
    MyModel(QObject* parent);

    int rowCount(const QModelIndex &parent) const;

//    QModelIndex index(int row, int column, const QModelIndex &parent) const;

    bool insertRows(int row, int count, const QModelIndex &parent);

    QVariant data(const QModelIndex &index, int role) const;

    void loadImages(QString filename);

private:
    QVector<QPixmap> m_pictures;
    QVector<QIcon> m_pics;
};

#endif // MYMODEL_H
