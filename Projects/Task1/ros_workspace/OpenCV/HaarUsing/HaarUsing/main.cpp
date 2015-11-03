#include <iostream>
#include <iomanip>
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/gpu/gpu.hpp"

using namespace std;
using namespace cv;
using namespace cv::gpu;


// 0) Преобразование изображения в GRAY
template<class T>
void convertAndResize(const T& src, T& gray, T& resized, double scale)
{
    if (src.channels() == 3)
    {
        cvtColor( src, gray, CV_BGR2GRAY );
    }
    else
    {
        gray = src;
    }

    Size sz(cvRound(gray.cols * scale), cvRound(gray.rows * scale));

    if (scale != 1)
    {
        resize(gray, resized, sz);
    }
    else
    {
        resized = gray;
    }
}

int main()
{
    // 1) Проверка доступных GPU
    if(getCudaEnabledDeviceCount() > 0) {

        // 2) Вывод информации о GPU
        cv::gpu::printShortCudaDeviceInfo(cv::gpu::getDevice());

        // 3) Создание классификатора
        CascadeClassifier_GPU cascade_gpu;
        cascade_gpu.load("/home/bulat/CUDA2_OpenCV_ROS/Project/OpenCV/data/cascade.xml");

        // 4) Загрузка изображения для проверки
        Mat image = imread("/home/bulat/CUDA2_OpenCV_ROS/Project/OpenCV/pos/p1.bmp");

        // 5) Создание вспомогательных объектов Mat
        Mat ImageBuf, resized_cpu;

        // 6) Создание объекта GpuMat и копирование изображения в память GPU
        GpuMat gpuImage(image);

        // 7) Создание вспомогательных объектов Mat
        GpuMat gpuImageBuf, gray_gpu, resized_gpu;

        // 8) Параметры каскада
        double scaleFactor = 1.0;
        bool findLargestObject = true;

        // 9) Отображение изображения
        imshow("Image",image);
        waitKey(0);

        // 10) Подготовка изображения
        convertAndResize(gpuImage,gray_gpu,resized_gpu,scaleFactor);

        // 11) Установка параметра
        cascade_gpu.findLargestObject = findLargestObject;

        // 12) Параметры искомого фрагмента
        Size minSize;
        minSize.height = 60;
        minSize.width = 60;

        int detections_num;
        // 13) Поиск объекта
        detections_num = cascade_gpu.detectMultiScale(resized_gpu,gpuImageBuf, 1.2, 1, minSize);

        // 14) Выгрузка объектов из GPU
        gpuImageBuf.colRange(0,detections_num).download(ImageBuf);
        resized_gpu.download(resized_cpu);

        cout << "Num: " << detections_num << endl;

        // 15) Выделение найденного объекта в прямоугольник
        for (int i = 0; i < detections_num; ++i)
        {
        rectangle(resized_cpu, ImageBuf.ptr<cv::Rect>()[i], Scalar(15));
        }

        // 16) Отображение найденного фрагмента
        imshow("Result",resized_cpu);

        // 17) Запись
        imwrite("imgOut.png",resized_cpu);
    }
    waitKey(0);

    return 0;
}

