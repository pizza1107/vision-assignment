
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <array>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <regex>
#include <filesystem>
#include <utility>

#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

// ==========================================
// 1. Data structures
// ==========================================

struct PreprocessResult {
    std::vector<Ort::Float16_t> tensor;
    float scale;
    int padLeft;
    int padTop;
};

struct Detection {
    std::array<cv::Point2f, 4> corners;
    cv::Rect2f box;
    float confidence = 0;
    int color = 0;
    int pattern = 0;
};

struct Calibration {
    cv::Mat cameraMatrix;
    cv::Mat distortion;
};

struct PoseResult {
    bool valid = false;
    std::array<cv::Point3d, 4> cornersCamera{};
    double reprojectionError = 0;
};

struct Arguments {
    std::string model;
    std::string video;
    std::string calibration;
    std::string output;
};

const std::array<std::string, 4> colorNames = {
    "blue", "red", "neutral", "purple"
};

const std::array<std::string, 9> patternNames = {
    "sentry", "1", "2", "3", "4",
    "5", "outpost", "base_small", "base_big"
};

// ==========================================
// 2. Command-line arguments
// ==========================================

Arguments parseArguments(int argc, char* argv[]) {
    Arguments args;

    for (int i = 1; i < argc; ++i) {
        std::string key = argv[i];

        if (i + 1 >= argc) {
            throw std::runtime_error(
                "Missing value for argument: " + key
            );
        }

        std::string value = argv[++i];

        if (key == "--model") {
            args.model = value;
        } else if (key == "--video") {
            args.video = value;
        } else if (key == "--calibration") {
            args.calibration = value;
        } else if (key == "--output") {
            args.output = value;
        } else {
            throw std::runtime_error(
                "Unknown argument: " + key
            );
        }
    }

    if (args.model.empty() ||
        args.video.empty() ||
        args.calibration.empty() ||
        args.output.empty()) {
        throw std::runtime_error(
            "Usage: detection --model FILE "
            "--video FILE --calibration FILE "
            "--output FILE"
        );
    }

    return args;
}

// ==========================================
// 3. Image preprocessing
// ==========================================

PreprocessResult preprocess(const cv::Mat& frame) {
    if (frame.empty()) {
        throw std::runtime_error("Empty frame");
    }

    constexpr int size = 640;

    float scale = std::min(
        static_cast<float>(size) / frame.cols,
        static_cast<float>(size) / frame.rows
    );

    int newW = static_cast<int>(
        std::round(frame.cols * scale)
    );
    int newH = static_cast<int>(
        std::round(frame.rows * scale)
    );

    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(newW, newH));

    int left = (size - newW) / 2;
    int top = (size - newH) / 2;
    int right = size - newW - left;
    int bottom = size - newH - top;

    cv::Mat padded;
    cv::copyMakeBorder(
        resized, padded,
        top, bottom, left, right,
        cv::BORDER_CONSTANT,
        cv::Scalar(114, 114, 114)
    );

    cv::Mat rgb;
    cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);

    std::vector<Ort::Float16_t> tensor(
        3 * size * size
    );

    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            cv::Vec3b pixel = rgb.at<cv::Vec3b>(y, x);

            for (int c = 0; c < 3; ++c) {
                int index = c * size * size + y * size + x;

                tensor[index] = Ort::Float16_t(
                    static_cast<float>(pixel[c]) / 255.0f
                );
            }
        }
    }

    return {std::move(tensor), scale, left, top};
}

// ==========================================
// 4. Math utilities
// ==========================================

float sigmoid(float x) {
    if (x >= 0.0f) {
        return 1.0f / (1.0f + std::exp(-x));
    }
    float e = std::exp(x);
    return e / (1.0f + e);
}

int argmax(const float* data, int count) {
    return static_cast<int>(
        std::max_element(data, data + count) - data
    );
}

float calculateIoU(
    const cv::Rect2f& a,
    const cv::Rect2f& b
) {
    float intersection = (a & b).area();
    float unionArea =
        a.area() + b.area() - intersection;

    return unionArea > 0 ? intersection / unionArea : 0;
}

// ==========================================
// 5. Decode model predictions
// ==========================================

std::vector<Detection> decode(
    const float* predictions,
    int count,
    const PreprocessResult& prep
) {
    std::vector<Detection> candidates;

    for (int i = 0; i < count; ++i) {
        const float* pred = predictions + i * 22;

        float confidence = sigmoid(pred[8]);

        if (!std::isfinite(confidence) ||
            confidence < 0.5f) {
            continue;
        }

        Detection det{};
        det.confidence = confidence;
        det.color = argmax(pred + 9, 4);
        det.pattern = argmax(pred + 13, 9);

        float minX = std::numeric_limits<float>::max();
        float minY = std::numeric_limits<float>::max();
        float maxX = std::numeric_limits<float>::lowest();
        float maxY = std::numeric_limits<float>::lowest();

        bool valid = true;

        for (int j = 0; j < 4; ++j) {
            float x =
                (pred[2 * j] - prep.padLeft) / prep.scale;
            float y =
                (pred[2 * j + 1] - prep.padTop) / prep.scale;

            if (!std::isfinite(x) ||
                !std::isfinite(y)) {
                valid = false;
                break;
            }

            det.corners[j] = {x, y};

            minX = std::min(minX, x);
            minY = std::min(minY, y);
            maxX = std::max(maxX, x);
            maxY = std::max(maxY, y);
        }

        if (!valid || minX >= maxX || minY >= maxY) {
            continue;
        }

        det.box = {
            minX, minY,
            maxX - minX,
            maxY - minY
        };

        candidates.push_back(det);
    }

    return candidates;
}

// ==========================================
// 6. Non-Maximum Suppression
// ==========================================

std::vector<Detection> applyNMS(
    const std::vector<Detection>& candidates,
    float threshold = 0.5f
) {
    std::vector<int> order(candidates.size());
    std::iota(order.begin(), order.end(), 0);

    std::sort(order.begin(), order.end(),
        [&](int a, int b) {
            return candidates[a].confidence >
                   candidates[b].confidence;
        }
    );

    std::vector<Detection> result;
    std::vector<bool> removed(candidates.size(), false);

    for (size_t i = 0; i < order.size(); ++i) {
        if (removed[i]) continue;

        const auto& best = candidates[order[i]];
        result.push_back(best);

        for (size_t j = i + 1; j < order.size(); ++j) {
            if (removed[j]) continue;

            const auto& other = candidates[order[j]];

            if (calculateIoU(best.box, other.box) >
                threshold) {
                removed[j] = true;
            }
        }
    }

    return result;
}

// ==========================================
// 7. Camera calibration
// ==========================================

std::vector<double> extractNumbers(
    const std::string& line
) {
    static const std::regex pattern(
        R"([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)"
    );

    std::vector<double> values;

    auto begin = std::sregex_iterator(
        line.begin(), line.end(), pattern
    );

    for (auto it = begin;
         it != std::sregex_iterator(); ++it) {
        values.push_back(std::stod(it->str()));
    }

    return values;
}

Calibration loadCalibration(const std::string& path) {
    std::ifstream file(path);

    if (!file) {
        throw std::runtime_error(
            "Cannot open calibration file: " + path
        );
    }

    std::vector<double> matrixValues;
    std::vector<double> distortionValues;

    enum class Section {
        None, Matrix, Distortion
    };

    Section section = Section::None;
    std::string line;

    while (std::getline(file, line)) {
        if (line.find("camera_matrix:") !=
            std::string::npos) {
            section = Section::Matrix;
            continue;
        }

        if (line.find("distort_coeffs:") !=
            std::string::npos) {
            section = Section::Distortion;
            continue;
        }

        auto pos = line.find_first_not_of(" \t\r");

        if (pos == std::string::npos ||
            line[pos] == '#') {
            continue;
        }

        if (line[pos] != '-') {
            section = Section::None;
            continue;
        }

        auto values = extractNumbers(line);

        if (section == Section::Matrix) {
            matrixValues.insert(
                matrixValues.end(),
                values.begin(), values.end()
            );
        } else if (section == Section::Distortion) {
            distortionValues.insert(
                distortionValues.end(),
                values.begin(), values.end()
            );
        }
    }

    if (matrixValues.size() != 9 ||
        distortionValues.size() != 5) {
        throw std::runtime_error(
            "Invalid calibration format"
        );
    }

    Calibration result;

    result.cameraMatrix = cv::Mat(3, 3, CV_64F);
    result.distortion = cv::Mat(1, 5, CV_64F);

    for (int i = 0; i < 9; ++i) {
        result.cameraMatrix.at<double>(
            i / 3, i % 3
        ) = matrixValues[i];
    }

    for (int i = 0; i < 5; ++i) {
        result.distortion.at<double>(0, i) =
            distortionValues[i];
    }

    return result;
}

// ==========================================
// 8. PnP pose estimation
// ==========================================

PoseResult estimatePose(
    const Detection& det,
    const Calibration& calibration
) {
    PoseResult result;

    bool large = det.pattern == 1 || det.pattern == 8;

    double w = large ? 0.230 : 0.135;
    double h = 0.056;

    std::vector<cv::Point3d> objectPoints = {
        {-w / 2, -h / 2, 0},
        {-w / 2,  h / 2, 0},
        { w / 2,  h / 2, 0},
        { w / 2, -h / 2, 0}
    };

    std::vector<cv::Point2f> imagePoints(
        det.corners.begin(), det.corners.end()
    );

    std::vector<cv::Mat> rvecs, tvecs;

    int count = cv::solvePnPGeneric(
        objectPoints,
        imagePoints,
        calibration.cameraMatrix,
        calibration.distortion,
        rvecs,
        tvecs,
        false,
        cv::SOLVEPNP_IPPE
    );

    double bestError =
        std::numeric_limits<double>::infinity();

    for (int k = 0; k < count; ++k) {
        cv::Mat R;
        cv::Rodrigues(rvecs[k], R);

        cv::Vec3d t(
            tvecs[k].at<double>(0),
            tvecs[k].at<double>(1),
            tvecs[k].at<double>(2)
        );

        if (!std::isfinite(t[0]) ||
            !std::isfinite(t[1]) ||
            !std::isfinite(t[2])) {
            continue;
        }

        std::array<cv::Point3d, 4> cornersCamera;
        bool valid = true;

        for (size_t i = 0; i < 4; ++i) {
            const auto& p = objectPoints[i];

            cv::Vec3d transformed(
                R.at<double>(0, 0) * p.x +
                R.at<double>(0, 1) * p.y +
                R.at<double>(0, 2) * p.z,

                R.at<double>(1, 0) * p.x +
                R.at<double>(1, 1) * p.y +
                R.at<double>(1, 2) * p.z,

                R.at<double>(2, 0) * p.x +
                R.at<double>(2, 1) * p.y +
                R.at<double>(2, 2) * p.z
            );

            transformed += t;

            if (!std::isfinite(transformed[0]) ||
                !std::isfinite(transformed[1]) ||
                !std::isfinite(transformed[2]) ||
                transformed[2] <= 0) {
                valid = false;
                break;
            }

            cornersCamera[i] = {
                transformed[0],
                transformed[1],
                transformed[2]
            };
        }

        if (!valid) continue;

        std::vector<cv::Point2d> projected;

        cv::projectPoints(
            objectPoints,
            rvecs[k],
            tvecs[k],
            calibration.cameraMatrix,
            calibration.distortion,
            projected
        );

        double sumSquared = 0;

        for (size_t i = 0; i < 4; ++i) {
            double dx =
                projected[i].x - imagePoints[i].x;
            double dy =
                projected[i].y - imagePoints[i].y;

            sumSquared += dx * dx + dy * dy;
        }

        double rms = std::sqrt(sumSquared / 4.0);

        if (std::isfinite(rms) && rms < bestError) {
            bestError = rms;
            result.valid = true;
            result.reprojectionError = rms;
            result.cornersCamera = cornersCamera;
        }
    }

    return result;
}

// ==========================================
// 9. JSONL output
// ==========================================

void writeFrame(
    std::ostream& out,
    int frameIndex,
    const std::vector<Detection>& detections,
    const Calibration& calibration
) {
    // Use the assignment's fixed 30 FPS.
    double timestamp = frameIndex / 30.0;

    out << "{\"frame_index\":" << frameIndex
        << ",\"timestamp_s\":" << timestamp
        << ",\"detections\":[";

    for (size_t i = 0; i < detections.size(); ++i) {
        if (i > 0) out << ",";

        const Detection& det = detections[i];
        PoseResult pose = estimatePose(det, calibration);

        out << "{\"color\":\""
            << colorNames[det.color]
            << "\",\"pattern\":\""
            << patternNames[det.pattern]
            << "\",\"confidence\":"
            << det.confidence;

        // 2D corners in original image pixels
        out << ",\"corners_px\":[";

        for (size_t j = 0; j < 4; ++j) {
            if (j > 0) out << ",";

            out << "[" << det.corners[j].x
                << "," << det.corners[j].y << "]";
        }

        out << "]";

        // 3D corners in camera coordinates
        if (pose.valid) {
            out << ",\"corners_camera_m\":[";

            for (size_t j = 0; j < 4; ++j) {
                if (j > 0) out << ",";

                const auto& p = pose.cornersCamera[j];

                out << "[" << p.x
                    << "," << p.y
                    << "," << p.z << "]";
            }

            out << "]";

            out << ",\"reprojection_error_px\":"
                << pose.reprojectionError;

        } else {
            // Preserve detection when PnP fails.
            out << ",\"corners_camera_m\":null";
            out << ",\"reprojection_error_px\":null";
        }

        out << "}";
    }

    out << "]}\n";
}

// ==========================================
// 10. Main
// ==========================================

int main(int argc, char* argv[]) {
    try {
        Arguments args = parseArguments(argc, argv);

        Calibration calibration =
            loadCalibration(args.calibration);

        Ort::Env env(
            ORT_LOGGING_LEVEL_WARNING,
            "RoboMasterDetection"
        );

        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(4);
        options.SetGraphOptimizationLevel(
            GraphOptimizationLevel::ORT_ENABLE_ALL
        );

        // Convert model path for Windows ONNX Runtime.
        std::wstring modelPath =
            std::filesystem::path(args.model).wstring();

        Ort::Session session(
            env,
            modelPath.c_str(),
            options
        );

        // Verify model input and output types.
        auto inputInfo =
            session.GetInputTypeInfo(0)
                   .GetTensorTypeAndShapeInfo();

        auto outputInfo =
            session.GetOutputTypeInfo(0)
                   .GetTensorTypeAndShapeInfo();

        if (inputInfo.GetElementType() !=
                ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16 ||
            outputInfo.GetElementType() !=
                ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
            throw std::runtime_error(
                "Unexpected model tensor types"
            );
        }

        cv::VideoCapture cap(args.video);

        if (!cap.isOpened()) {
            throw std::runtime_error(
                "Failed to open video: " + args.video
            );
        }

        std::ofstream output(args.output);

        if (!output) {
            throw std::runtime_error(
                "Cannot open output: " + args.output
            );
        }

        output << std::setprecision(12);

        const char* inputNames[] = {"images"};
        const char* outputNames[] = {"output"};

        const std::array<int64_t, 4> inputShape = {
            1, 3, 640, 640
        };

        Ort::MemoryInfo memoryInfo =
            Ort::MemoryInfo::CreateCpu(
                OrtArenaAllocator,
                OrtMemTypeDefault
            );

        int frameIndex = 0;
        int totalDetections = 0;
        cv::Mat frame;

        std::cout << "Processing: "
                  << args.video << '\n';

        // Process every frame until the video ends.
        while (cap.read(frame)) {

            PreprocessResult prep = preprocess(frame);

            Ort::Value inputTensor =
                Ort::Value::CreateTensor<Ort::Float16_t>(
                    memoryInfo,
                    prep.tensor.data(),
                    prep.tensor.size(),
                    inputShape.data(),
                    inputShape.size()
                );

            auto outputs = session.Run(
                Ort::RunOptions{nullptr},
                inputNames,
                &inputTensor,
                1,
                outputNames,
                1
            );

            auto info =
                outputs[0].GetTensorTypeAndShapeInfo();

            auto shape = info.GetShape();

            if (shape.size() != 3 ||
                shape[0] != 1 ||
                shape[1] != 25200 ||
                shape[2] != 22 ||
                info.GetElementType() !=
                    ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
                throw std::runtime_error(
                    "Unexpected model output"
                );
            }

            const float* predictions =
                outputs[0].GetTensorData<float>();

            auto candidates = decode(
                predictions,
                static_cast<int>(shape[1]),
                prep
            );

            auto detections = applyNMS(candidates);

            // Write exactly one JSON object per frame.
            writeFrame(
                output,
                frameIndex,
                detections,
                calibration
            );

            if (!output) {
                throw std::runtime_error(
                    "Failed to write JSONL output"
                );
            }

            totalDetections +=
                static_cast<int>(detections.size());

            ++frameIndex;

            if (frameIndex % 30 == 0) {
                std::cout << "Processed "
                          << frameIndex
                          << " frames\n";
            }
        }

        output.close();

        std::cout << "\nCompleted!\n";
        std::cout << "Total frames: "
                  << frameIndex << '\n';
        std::cout << "Total detections: "
                  << totalDetections << '\n';
        std::cout << "Saved to: "
                  << args.output << '\n';

    } catch (const Ort::Exception& e) {
        std::cerr << "ONNX Runtime error: "
                  << e.what() << '\n';
        return 1;

    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV error: "
                  << e.what() << '\n';
        return 1;

    } catch (const std::exception& e) {
        std::cerr << "Error: "
                  << e.what() << '\n';
        return 1;
    }

    return 0;
}
