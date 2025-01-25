// face_detector.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class FaceDetector : public rclcpp::Node
{
public:
    FaceDetector()
    : Node("face_detector")
    {
        // Astraカメラのイメージトピックに合わせて変更してください
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/astra_camera/image_raw",
            10,
            std::bind(&FaceDetector::image_callback, this, std::placeholders::_1)
        );

        // Haar Cascadeの分類器をロード
        // face_detector.cppでは、cv::samples::findFileを使用してファイルを検索していますが、環境によってはパスを明示的に指定する必要があります.
        if (!face_cascade_.load(cv::samples::findFile("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"))) {
            RCLCPP_ERROR(this->get_logger(), "Haar Cascade XMLファイルのロードに失敗しました。");
            throw std::runtime_error("Haar Cascade XMLファイルのロードに失敗");
        }

        RCLCPP_INFO(this->get_logger(), "Face Detector Node has been started.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            // ROSイメージメッセージをOpenCV画像に変換
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // グレースケールに変換
        cv::Mat gray;
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

        // 顔検知
        std::vector<cv::Rect> faces;
        face_cascade_.detectMultiScale(gray, faces, 1.1, 5, 0, cv::Size(30, 30));

        if (!faces.empty()) {
            RCLCPP_INFO(this->get_logger(), "顔検知");
        }

        // （オプション）検出した顔に矩形を描画して表示
        for (size_t i = 0; i < faces.size(); i++) {
            cv::rectangle(cv_ptr->image, faces[i], cv::Scalar(0, 255, 0), 2);
        }

        // ウィンドウに表示（必要に応じてコメントアウト）
        cv::imshow("Face Detection", cv_ptr->image);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::CascadeClassifier face_cascade_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<FaceDetector>());
    }
    catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
