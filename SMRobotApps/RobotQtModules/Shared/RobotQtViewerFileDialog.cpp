#include "RobotQtViewerFileDialog.h"

namespace robot_qt_viewer
{
    QFileDialog::Options fastFileDialogOptions(QFileDialog::Options options)
    {
        options |= QFileDialog::DontUseNativeDialog;
        options |= QFileDialog::DontUseCustomDirectoryIcons;
        return options;
    }

    QString getOpenFileName(
        QWidget* parent,
        const QString& caption,
        const QString& dir,
        const QString& filter,
        QString* selectedFilter,
        QFileDialog::Options options)
    {
        return QFileDialog::getOpenFileName(
            parent,
            caption,
            dir,
            filter,
            selectedFilter,
            fastFileDialogOptions(options));
    }

    QString getSaveFileName(
        QWidget* parent,
        const QString& caption,
        const QString& dir,
        const QString& filter,
        QString* selectedFilter,
        QFileDialog::Options options)
    {
        return QFileDialog::getSaveFileName(
            parent,
            caption,
            dir,
            filter,
            selectedFilter,
            fastFileDialogOptions(options));
    }
}
