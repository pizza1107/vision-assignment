
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <array>
#include <vector>

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr
            << "Usage: visualize VIDEO JSONL OUTPUT\n";
        return 1;
    }

    cv::VideoCapture cap(argv[1]);
    std::ifstream jsonFile(argv[2]);

    if (!cap.isOpened() || !jsonFile) {
        std::cerr << "Cannot open video or JSONL\n";
        return 1;
    }

    double fps = cap.get(cv::CAP_PROP_FPS);
    int width = static_cast<int>(
        cap.get(cv::CAP_PROP_FRAME_WIDTH)
    );
    int height = static_cast<int>(
        cap.get(cv::CAP_PROP_FRAME_HEIGHT)
    );

    cv::VideoWriter writer(
        argv[3],
        cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
        fps,
        cv::Size(width, height)
    );

    if (!writer.isOpened()) {
        std::cerr << "Cannot create output video\n";
        return 1;
    }

    cv::Mat frame;
    std::string line;
    int frameIndex = 0;

    while (cap.read(frame)) {
        if (!std::getline(jsonFile, line)) {
            std::cerr << "Missing JSONL frame\n";
            return 1;
        }

        // Extract corners_px arrays from each detection.
        // This matches the known output format from main.cpp.
        size_t pos = 0;

        while ((pos = line.find(
            "\"corners_px\":", pos
        )) != std::string::npos) {

            size_t start = line.find('[', pos);
            if (start == std::string::npos) break;

            size_t end = line.find("]]", start);
            if (end == std::string::npos) break;

            std::string cornerText =
                line.substr(start, end - start + 2);

            // Read numeric coordinates
            std::vector<double> values;
            std::string number;

            for (char c : cornerText) {
                if ((c >= '0' && c <= '9') ||
                    c == '-' || c == '+' ||
                    c == '.' || c == 'e' || c == 'E') {
                    number += c;
                } else if (!number.empty()) {
                    values.push_back(std::stod(number));
                    number.clear();
                }
            }

            if (values.size() == 8) {
                std::vector<cv::Point> points;

                for (int i = 0; i < 4; ++i) {
                    points.emplace_back(
                        cvRound(values[i * 2]),
                        cvRound(values[i * 2 + 1])
                    );
                }

                cv::polylines(
                    frame,
                    points,
                    true,
                    cv::Scalar(0, 255, 0),
                    2,
                    cv::LINE_AA
                );

                for (const auto& point : points) {
                    cv::circle(
                        frame, point, 4,
                        cv::Scalar(0, 0, 255), -1
                    );
                }
            }

            pos = end + 2;
        }

        cv::putText(
            frame,
            "Frame: " + std::to_string(frameIndex),
            cv::Point(25, 45),
            cv::FONT_HERSHEY_SIMPLEX,
            1.0,
            cv::Scalar(255, 255, 255),
            2
        );

        writer.write(frame);
        ++frameIndex;
    }

    writer.release();
    cap.release();

    std::cout << "Visualization completed!\n";
    std::cout << "Frames: " << frameIndex << '\n';
    std::cout << "Saved to: " << argv[3] << '\n';

    return 0;
}
