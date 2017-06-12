#include "mymodel.hpp"
#include <QtCore>

MyModel::MyModel(QObject* parent) : QAbstractListModel(parent)
{

}

int MyModel::rowCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return m_pictures.size();
}

bool MyModel::insertRows(int row, int count, const QModelIndex &parent)
{
    Q_UNUSED(count);
    Q_UNUSED(parent);

    beginInsertRows(parent, row, row + count);
    endInsertRows();
    return true;
}

QVariant MyModel::data(const QModelIndex &index, int role) const
{
    int row = index.row();

    if(row >= m_pictures.size()) {
        qWarning() << "data query out of range";
        return QVariant();
    }

    switch (role) {
    case Qt::DecorationRole:
        return m_pictures.at(row);
    default:
        break;
    }

    return QVariant();
}

void MyModel::loadImages(QString filename)
{
    QFileInfoList allfiles;
    QFileInfo info;
    info.setFile(filename);
    QDir dir = info.dir();

    QStringList filter;
    filter << "*.png" << "*.jpg";
    allfiles = dir.entryInfoList(filter, QDir::Files, QDir::Name);

//    foreach (const QFileInfo& info, m_fileinfo) {
//        qDebug() << info.absoluteFilePath();
//        m_pictures.push_back(QPixmap(info.absoluteFilePath()).scaledToHeight(100));
//    }

    m_pictures.resize(allfiles.size());
    m_pics.resize(allfiles.size());
    for(int i = 0; i < allfiles.size(); ++i) {
       m_pictures[i] = QPixmap(allfiles[i].absoluteFilePath()).scaledToHeight(100);
    }


    insertRows(0, allfiles.size(), QModelIndex());

    return;
}

