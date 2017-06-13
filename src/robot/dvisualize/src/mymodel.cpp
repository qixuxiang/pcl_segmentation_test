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
        return m_pictures.at(row).scaledToWidth(100);
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
    m_picname.resize(allfiles.size());
    for(int i = 0; i < allfiles.size(); ++i) {
       m_pictures[i] = QPixmap(allfiles[i].absoluteFilePath());
       m_picname[i] = allfiles[i].fileName();

    }



    insertRows(0, allfiles.size(), QModelIndex());
}

const QPixmap MyModel::getImage(int row) const
{
    if(row < 0 || row >= m_pictures.size()) {
        qWarning() << "index out of range" << endl;
       return QPixmap();
    }
    return m_pictures.at(row);
}

const QPixmap MyModel::getImage() const
{
    return getImage(m_currentIndex);
}

void MyModel::setCurrentIndex(int idx)
{
    m_currentIndex = idx;
}

QPoint MyModel::getPlatAngle(int index)
{
    // extremly ugly
    QString filename = m_picname.at(index);
//    "p_60_y_45 1497349252533223778.png"


    auto list = filename.split(' ');
    auto s = list[0].split('_');

    auto pitch = s[1].toInt();
    auto yaw = s[3].toInt();

    return QPoint(pitch, yaw);
}

