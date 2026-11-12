
#include <iostream>
#include <vector>
#include <array>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

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
    float confidence;
    int color;
    int pattern;
};

// ==========================================
// 2. Image preprocessing
// ==========================================

PreprocessResult preprocess(const cv::Mat& frame) {
    constexpr int inputSize = 640;

    // Calculate scaling factor
    float scale = std::min(
        static_cast<float>(inputSize) / frame.cols,
        static_cast<float>(inputSize) / frame.rows
    );

    int newW = static_cast<int>(
        std::round(frame.cols * scale)
    );
    int newH = static_cast<int>(
        std::round(frame.rows * scale)
    );

    // Resize image
    cv::Mat resized;
    cv::resize(
        frame, resized,
        cv::Size(newW, newH)
    );

    // Calculate padding
    int padLeft = (inputSize - newW) / 2;
    int padTop = (inputSize - newH) / 2;
    int padRight = inputSize - newW - padLeft;
    int padBottom = inputSize - newH - padTop;

    // Letterbox padding
    cv::Mat padded;
    cv::copyMakeBorder(
        resized, padded,
        padTop, padBottom,
        padLeft, padRight,
        cv::BORDER_CONSTANT,
        cv::Scalar(114, 114, 114)
    );

    // BGR -> RGB
    cv::Mat rgb;
    cv::cvtColor(
        padded, rgb,
        cv::COLOR_BGR2RGB
    );

    // Normalize and convert HWC -> CHW (Float16)
    std::vector<Ort::Float16_t> tensor(
        3 * inputSize * inputSize
    );

    for (int y = 0; y < inputSize; ++y) {
        for (int x = 0; x < inputSize; ++x) {

            cv::Vec3b pixel =
                rgb.at<cv::Vec3b>(y, x);

            for (int c = 0; c < 3; ++c) {
                int index =
                    c * inputSize * inputSize
                    + y * inputSize + x;

                float value =
                    static_cast<float>(pixel[c]) / 255.0f;

                tensor[index] = Ort::Float16_t(value);
            }
        }
    }

    return {
        std::move(tensor),
        scale,
        padLeft,
        padTop
    };
}

// ==========================================
// 3. Sigmoid and Argmax
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
        std::max_element(
            data, data + count
        ) - data
    );
}

// ==========================================
// 4. IoU calculation
// ==========================================

float calculateIoU(
    const cv::Rect2f& a,
    const cv::Rect2f& b
) {
    float intersection = (a & b).area();

    float unionArea =
        a.area() + b.area() - intersection;

    if (unionArea <= 0.0f) {
        return 0.0f;
    }

    return intersection / unionArea;
}

// ==========================================
// 5. Decode model predictions
// ==========================================

std::vector<Detection> decode(
    const float* predictions,
    int numPredictions,
    float scale,
    int padLeft,
    int padTop
) {
    constexpr int attributes = 22;
    constexpr float confidenceThreshold = 0.5f;

    std::vector<Detection> candidates;

    for (int i = 0; i < numPredictions; ++i) {

        const float* pred =
            predictions + i * attributes;

        // Objectness
        float confidence = sigmoid(pred[8]);

        if (!std::isfinite(confidence) ||
            confidence < confidenceThreshold) {
            continue;
        }

        Detection det{};

        det.confidence = confidence;

        // Color classification: indices 9-12
        det.color = argmax(pred + 9, 4);

        // Pattern classification: indices 13-21
        det.pattern = argmax(pred + 13, 9);

        float minX = std::numeric_limits<float>::max();
        float minY = std::numeric_limits<float>::max();
        float maxX = std::numeric_limits<float>::lowest();
        float maxY = std::numeric_limits<float>::lowest();

        bool valid = true;

        // Decode four corners
        for (int j = 0; j < 4; ++j) {

            // Restore original image coordinates
            float x =
                (pred[j * 2] - padLeft) / scale;

            float y =
                (pred[j * 2 + 1] - padTop) / scale;

            if (!std::isfinite(x) ||
                !std::isfinite(y)) {
                valid = false;
                break;
            }

            det.corners[j] = cv::Point2f(x, y);

            minX = std::min(minX, x);
            minY = std::min(minY, y);
            maxX = std::max(maxX, x);
            maxY = std::max(maxY, y);
        }

        if (!valid || maxX <= minX || maxY <= minY) {
            continue;
        }

        // Bounding rectangle from corners
        det.box = cv::Rect2f(
            minX,
            minY,
            maxX - minX,
            maxY - minY
        );

        candidates.push_back(det);
    }

    return candidates;
}

// ==========================================
// 6. Non-Maximum Suppression
// ==========================================

std::vector<Detection> applyNMS(
    const std::vector<Detection>& candidates,
    float iouThreshold = 0.5f
) {
    std::vector<int> indices(candidates.size());
    std::iota(indices.begin(), indices.end(), 0);

    // Sort by confidence (highest first)
    std::sort(
        indices.begin(),
        indices.end(),
        [&](int a, int b) {
            return candidates[a].confidence >
                   candidates[b].confidence;
        }
    );

    std::vector<Detection> result;
    std::vector<bool> suppressed(
        candidates.size(), false
    );

    for (size_t i = 0; i < indices.size(); ++i) {

        if (suppressed[i]) {
            continue;
        }

        const Detection& best =
            candidates[indices[i]];

        result.push_back(best);

        // Suppress overlapping candidates
        for (size_t j = i + 1; j < indices.size(); ++j) {

            if (suppressed[j]) {
                continue;
            }

            const Detection& other =
                candidates[indices[j]];

            if (calculateIoU(best.box, other.box) >
                iouThreshold) {

                suppressed[j] = true;
            }
        }
    }

    return result;
}

// ==========================================
// 7. Main program
// ==========================================

int main() {
    try {
        // Initialize ONNX Runtime
        Ort::Env env(
            ORT_LOGGING_LEVEL_WARNING,
            "RoboMasterDetection"
        );

        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(4);
        options.SetGraphOptimizationLevel(
            GraphOptimizationLevel::ORT_ENABLE_ALL
        );

        // Load pretrained model
        Ort::Session session(
            env,
            L"assets/0526.onnx",
            options
        );

        std::cout << "Model loaded successfully!\n";

        // Open competition video
        cv::VideoCapture cap("assets/small.mp4");

        if (!cap.isOpened()) {
            std::cerr << "Failed to open video!\n";
            return 1;
        }

        std::cout << "FPS: "
                  << cap.get(cv::CAP_PROP_FPS)
                  << '\n';

        std::cout << "Total frames: "
                  << cap.get(cv::CAP_PROP_FRAME_COUNT)
                  << '\n';

        // Read first frame
        cv::Mat frame;

        if (!cap.read(frame)) {
            std::cerr << "Failed to read frame!\n";
            return 1;
        }

        std::cout << "Frame size: "
                  << frame.cols << " x "
                  << frame.rows << '\n';

        // Preprocess frame
        PreprocessResult preprocessed =
            preprocess(frame);

        std::cout << "Preprocessing completed!\n";

        // Create ONNX input tensor
        std::array<int64_t, 4> inputShape = {
            1, 3, 640, 640
        };

        Ort::MemoryInfo memoryInfo =
            Ort::MemoryInfo::CreateCpu(
                OrtArenaAllocator,
                OrtMemTypeDefault
            );

        Ort::Value inputTensor =
            Ort::Value::CreateTensor<Ort::Float16_t>(
                memoryInfo,
                preprocessed.tensor.data(),
                preprocessed.tensor.size(),
                inputShape.data(),
                inputShape.size()
            );

        // Run inference
        const char* inputNames[] = {"images"};
        const char* outputNames[] = {"output"};

        auto outputs = session.Run(
            Ort::RunOptions{nullptr},
            inputNames,
            &inputTensor,
            1,
            outputNames,
            1
        );

        std::cout << "Inference completed!\n";

        // Check model output
        auto shapeInfo =
            outputs[0].GetTensorTypeAndShapeInfo();

        auto outputShape = shapeInfo.GetShape();

        if (outputShape.size() != 3 ||
            outputShape[0] != 1 ||
            outputShape[2] != 22 ||
            outputShape[1] <= 0 ||
            shapeInfo.GetElementType() !=
                ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {

            throw std::runtime_error(
                "Unexpected output tensor"
            );
        }

        std::cout << "Output shape: ";
        for (auto dim : outputShape) {
            std::cout << dim << " ";
        }
        std::cout << '\n';

        const float* predictions =
            outputs[0].GetTensorData<float>();

        // Decode predictions
        int numPredictions =
            static_cast<int>(outputShape[1]);

        auto candidates = decode(
            predictions,
            numPredictions,
            preprocessed.scale,
            preprocessed.padLeft,
            preprocessed.padTop
        );

        // Apply NMS
        auto detections = applyNMS(candidates);

        std::cout << "Candidates after filtering: "
                  << candidates.size() << '\n';

        std::cout << "Detections after NMS: "
                  << detections.size() << '\n';

        // Print detection results
        const std::array<std::string, 4> colors = {
            "blue", "red", "neutral", "purple"
        };

        const std::array<std::string, 9> patterns = {
            "sentry", "1", "2", "3", "4",
            "5", "outpost", "base_small", "base_big"
        };

        for (const auto& det : detections) {

            std::cout << "\nDetection:\n";

            std::cout << "  Confidence: "
                      << det.confidence << '\n';

            std::cout << "  Color: "
                      << colors[det.color] << '\n';

            std::cout << "  Pattern: "
                      << patterns[det.pattern] << '\n';

            std::cout << "  Corners:\n";

            for (const auto& pt : det.corners) {
                std::cout << "    ("
                          << pt.x << ", "
                          << pt.y << ")\n";
            }
        }

        cap.release();

        std::cout << "\nStep 8 completed!\n";

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
