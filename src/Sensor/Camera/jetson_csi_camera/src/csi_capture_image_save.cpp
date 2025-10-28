#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <ctime>
#include <sys/stat.h>
#include <iomanip>
#include <sstream>

class CSICameraCapture
{
public:
    CSICameraCapture(int sensor_id = 0, int width = 1280, int height = 720, int framerate = 30)
        : sensor_id_(sensor_id), width_(width), height_(height), framerate_(framerate), saved_count_(0)
    {
        pipeline_ = createGStreamerPipeline(sensor_id_, width_, height_, framerate_, 0);  // flip_method=0 (반전 없음)
    }

    bool initialize()
    {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "CSI 카메라 초기화 중..." << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "GStreamer Pipeline:" << std::endl;
        std::cout << pipeline_ << std::endl;
        std::cout << std::string(60, '=') << std::endl;

        cap_.open(pipeline_, cv::CAP_GSTREAMER);

        if (!cap_.isOpened())
        {
            std::cerr << "\n" << std::string(60, '=') << std::endl;
            std::cerr << "❌ Error: CSI 카메라 (센서 ID " << sensor_id_ << ")를 열 수 없습니다." << std::endl;
            std::cerr << std::string(60, '=') << std::endl;
            std::cerr << "문제 해결:" << std::endl;
            std::cerr << "1. 카메라 케이블이 제대로 연결되어 있는지 확인" << std::endl;
            std::cerr << "2. 다음 명령어로 카메라 테스트:" << std::endl;
            std::cerr << "   $ gst-launch-1.0 nvarguscamerasrc sensor-id=" << sensor_id_
                     << " ! nvoverlaysink" << std::endl;
            std::cerr << std::string(60, '=') << std::endl;
            return false;
        }

        std::cout << "\n✅ CSI 카메라 초기화 성공!" << std::endl;
        std::cout << "센서 ID: " << sensor_id_ << std::endl;
        std::cout << "해상도: " << width_ << " x " << height_ << std::endl;
        std::cout << "프레임레이트: " << framerate_ << " FPS" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "📷 조작법:" << std::endl;
        std::cout << "   SPACE BAR: 현재 프레임 저장" << std::endl;
        std::cout << "   ESC: 프로그램 종료" << std::endl;
        std::cout << std::string(60, '=') << std::endl;

        return true;
    }

    void run()
    {
        if (!cap_.isOpened())
        {
            std::cerr << "카메라가 초기화되지 않았습니다." << std::endl;
            return;
        }

        cv::Mat frame;
        int frame_count = 0;

        // 첫 프레임 읽기 테스트
        if (cap_.read(frame))
        {
            std::cout << "\n✅ 프레임 읽기 성공!" << std::endl;
            std::cout << "실제 프레임 크기: " << frame.cols << " x " << frame.rows << " (가로 x 세로)" << std::endl;
            std::cout << "채널 수: " << frame.channels() << std::endl;
            std::cout << "메가픽셀: " << (width_ * height_) / 1000000.0 << "MP" << std::endl;
            std::cout << "\n카메라 화면이 표시됩니다." << std::endl;
            std::cout << "SPACE BAR를 눌러 이미지를 저장하고, ESC 키를 눌러 종료하세요.\n" << std::endl;
        }
        else
        {
            std::cerr << "❌ Error: 프레임을 읽을 수 없습니다." << std::endl;
            return;
        }

        std::string window_name = "CSI Camera " + std::to_string(sensor_id_) +
                                  " - Tire Detection (" + std::to_string(width_) +
                                  "x" + std::to_string(height_) + ")";

        while (true)
        {
            if (!cap_.read(frame))
            {
                std::cerr << "⚠️  프레임을 읽을 수 없습니다." << std::endl;
                break;
            }

            frame_count++;

            // 순수 원본 프레임 복사 (저장용)
            cv::Mat clean_frame = frame.clone();

            // 화면 표시용 프레임 (오버레이 추가)
            cv::Mat display_frame = frame.clone();

            // 오버레이 정보 추가
            addOverlayInfo(display_frame);

            // Detection Area 그리기
            drawDetectionArea(display_frame);

            // 화면에 표시
            cv::imshow(window_name, display_frame);

            // 키 입력 처리
            int key = cv::waitKey(1) & 0xFF;
            if (key == 27) // ESC key - 종료
            {
                std::cout << "\nESC 키 감지 - 종료 중..." << std::endl;
                break;
            }
            else if (key == 32) // SPACE key - 이미지 저장
            {
                if (saveImage(clean_frame))
                {
                    saved_count_++;
                    // 저장 성공 메시지 표시
                    cv::Mat save_frame = display_frame.clone();
                    std::string save_text = "SAVED! (" + std::to_string(saved_count_) + ")";
                    cv::putText(save_frame, save_text,
                               cv::Point(width_ / 2 - 100, height_ / 2),
                               cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 255, 0), 3);
                    cv::imshow(window_name, save_frame);
                    cv::waitKey(500); // 0.5초 동안 저장 메시지 표시
                }
            }
        }

        // 리소스 정리
        cap_.release();
        cv::destroyAllWindows();

        // 통계 출력
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "📊 세션 통계:" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "   총 처리된 프레임: " << frame_count << "개" << std::endl;
        std::cout << "   저장된 이미지: " << saved_count_ << "개" << std::endl;
        if (saved_count_ > 0)
        {
            std::cout << "   저장 위치: ./images/" << getCurrentDate() << "/ 폴더" << std::endl;
        }
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "✅ CSI 카메라가 정상적으로 종료되었습니다." << std::endl;
    }

    ~CSICameraCapture()
    {
        if (cap_.isOpened())
        {
            cap_.release();
        }
        cv::destroyAllWindows();
    }

private:
    std::string createGStreamerPipeline(int sensor_id, int width, int height, int framerate, int flip_method = 0)
    {
        // 1920x1080으로 캡처하여 센서 중앙 사용, 그 후 원하는 해상도로 스케일
        std::ostringstream pipeline;
        pipeline << "nvarguscamerasrc sensor-id=" << sensor_id << " ! "
                 << "video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, "
                 << "framerate=(fraction)" << framerate << "/1, format=(string)NV12 ! "
                 << "nvvidconv flip-method=" << flip_method << " ! "
                 << "video/x-raw, width=(int)" << width << ", height=(int)" << height << ", format=(string)BGRx ! "
                 << "videoconvert ! "
                 << "video/x-raw, format=(string)BGR ! "
                 << "appsink drop=1";
        return pipeline.str();
    }

    std::string getCurrentDate()
    {
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%d");
        return oss.str();
    }

    std::string getCurrentTime()
    {
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%H%M%S");
        return oss.str();
    }

    std::string getCurrentDateTime()
    {
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
        return oss.str();
    }

    bool createDirectory(const std::string& path)
    {
        struct stat info;
        if (stat(path.c_str(), &info) != 0)
        {
            // 디렉토리가 없으면 생성
            if (mkdir(path.c_str(), 0755) == 0)
            {
                std::cout << "📁 새 폴더 생성: " << path << std::endl;
                return true;
            }
            return false;
        }
        return true;
    }

    bool saveImage(const cv::Mat& frame)
    {
        std::string base_dir = "images";
        std::string today = getCurrentDate();
        std::string save_dir = base_dir + "/" + today;

        // 디렉토리 생성
        createDirectory(base_dir);
        createDirectory(save_dir);

        // 파일명 생성
        std::string filename = "image_" + getCurrentTime() + ".jpg";
        std::string filepath = save_dir + "/" + filename;

        // 이미지 저장 (최고 품질)
        std::vector<int> params;
        params.push_back(cv::IMWRITE_JPEG_QUALITY);
        params.push_back(100);

        bool success = cv::imwrite(filepath, frame, params);

        if (success)
        {
            std::cout << "✅ 이미지 저장 완료: " << filepath << std::endl;
        }
        else
        {
            std::cout << "❌ 이미지 저장 실패: " << filepath << std::endl;
        }

        return success;
    }

    void addOverlayInfo(cv::Mat& frame)
    {
        // 카메라 정보
        std::string info_text = "CSI Camera (Sensor " + std::to_string(sensor_id_) + "): " +
                                std::to_string(width_) + "x" + std::to_string(height_);
        cv::putText(frame, info_text, cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        // FPS 정보
        std::string fps_text = "FPS: " + std::to_string(framerate_);
        cv::putText(frame, fps_text, cv::Point(10, 65),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        // 저장된 이미지 수
        std::string save_text = "Saved: " + std::to_string(saved_count_) + " images";
        cv::putText(frame, save_text, cv::Point(10, 100),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        // 조작법 안내
        std::string help_text = "SPACE: Save Image | ESC: Exit";
        cv::putText(frame, help_text, cv::Point(10, height_ - 20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

        // 현재 시간
        std::string time_text = "Time: " + getCurrentDateTime();
        cv::putText(frame, time_text, cv::Point(width_ - 350, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
    }

    void drawDetectionArea(cv::Mat& frame)
    {
        int center_x = width_ / 2;
        int center_y = height_ / 2;

        // ROI 크기 계산 (화면의 30%)
        int roi_width = static_cast<int>(width_ * 0.3);
        int roi_height = static_cast<int>(height_ * 0.3);

        // 크기 제한
        roi_width = std::max(150, std::min(roi_width, width_ / 2));
        roi_height = std::max(150, std::min(roi_height, height_ / 2));

        // 정사각형으로 만들기
        int roi_size = std::min(roi_width, roi_height);

        // Detection Area 사각형 그리기
        cv::rectangle(frame,
                     cv::Point(center_x - roi_size, center_y - roi_size),
                     cv::Point(center_x + roi_size, center_y + roi_size),
                     cv::Scalar(255, 0, 0), 2);

        // Detection Area 텍스트
        double font_scale = std::max(0.4, std::min(1.0, width_ / 1920.0));
        std::string area_text = "DETECTION AREA";
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(area_text, cv::FONT_HERSHEY_SIMPLEX,
                                             font_scale, 2, &baseline);
        int text_x = center_x - text_size.width / 2;
        int text_y = center_y - roi_size - 10;

        cv::putText(frame, area_text, cv::Point(text_x, text_y),
                   cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(255, 0, 0), 2);
    }

    cv::VideoCapture cap_;
    std::string pipeline_;
    int sensor_id_;
    int width_;
    int height_;
    int framerate_;
    int saved_count_;
};

int main(int argc, char** argv)
{
    try
    {
        int sensor_id = 0;

        // 커맨드 라인 인자로 센서 ID 받기 (옵션)
        if (argc > 1)
        {
            sensor_id = std::atoi(argv[1]);
        }

        CSICameraCapture camera(sensor_id, 1280, 720, 30);

        if (!camera.initialize())
        {
            return 1;
        }

        camera.run();

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "예외 발생: " << e.what() << std::endl;
        return 1;
    }
}
