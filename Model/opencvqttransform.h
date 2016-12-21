#ifndef OPENCVQTTRANSFORM_H
#define OPENCVQTTRANSFORM_H

#include <QPixmap>

class OpenCVQTTransform
{
public:
    OpenCVQTTransform() {}

    static cv::Mat QImageToCvMat(const QImage &inImage, bool inCloneImageData = true);

    static cv::Mat QPixmapToCvMat(const QPixmap &inPixmap, bool inCloneImageData = true);
};
/**
    If inImage exists for the lifetime of the resulting cv::Mat, pass false to inCloneImageData to share inImage's
    data with the cv::Mat directly
    NOTE: Format_RGB888 is an exception since we need to use a local QImage and thus must clone the data regardless
    NOTE: This does not cover all cases - it should be easy to add new ones as required.
 **/
inline cv::Mat OpenCVQTTransform::QImageToCvMat( const QImage &inImage, bool inCloneImageData){
    switch ( inImage.format() )
    {
       // 8-bit, 4 channel
       case QImage::Format_ARGB32:
       case QImage::Format_ARGB32_Premultiplied:
       {
          cv::Mat  mat( inImage.height(), inImage.width(),
                        CV_8UC4,
                        const_cast<uchar*>(inImage.bits()),
                        static_cast<size_t>(inImage.bytesPerLine())
                        );

          return (inCloneImageData ? mat.clone() : mat);
       }

       // 8-bit, 3 channel
       case QImage::Format_RGB32:
       case QImage::Format_RGB888:
       {
          if ( !inCloneImageData )
          {
             qWarning() << "ASM::QImageToCvMat() - Conversion requires cloning because we use a temporary QImage";
          }

          QImage   swapped;

          if ( inImage.format() == QImage::Format_RGB32 )
             swapped = inImage.convertToFormat( QImage::Format_RGB888 );

           swapped = inImage.rgbSwapped();

          return cv::Mat( swapped.height(), swapped.width(),
                          CV_8UC3,
                          const_cast<uchar*>(swapped.bits()),
                          static_cast<size_t>(swapped.bytesPerLine())
                          ).clone();
       }

       // 8-bit, 1 channel
       case QImage::Format_Indexed8:
       {
          cv::Mat  mat( inImage.height(), inImage.width(),
                        CV_8UC1,
                        const_cast<uchar*>(inImage.bits()),
                        static_cast<size_t>(inImage.bytesPerLine())
                        );

          return (inCloneImageData ? mat.clone() : mat);
       }

       default:
          qWarning() << "ASM::QImageToCvMat() - QImage format not handled in switch:" << inImage.format();
          break;
    }

    return cv::Mat();
 }
/**
    If inPixmap exists for the lifetime of the resulting cv::Mat, pass false to inCloneImageData to share inPixmap's data
    with the cv::Mat directly
    NOTE: Format_RGB888 is an exception since we need to use a local QImage and thus must clone the data regardless
**/
inline cv::Mat OpenCVQTTransform::QPixmapToCvMat(const QPixmap &inPixmap, bool inCloneImageData){
    return QImageToCvMat(inPixmap.toImage(), inCloneImageData);
}

#endif /// OPENCVQTTRANSFORM_H
