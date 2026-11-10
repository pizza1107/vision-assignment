# 装甲板检测与位姿估计

[中文](README.md) · [English](README.en.md)

这是两条可选路线之一。**检测与瞄准任选其一即可，不要求同时完成。**
阅读[主 README](../README.md)了解云台教具联调的故事背景。
本路线使用录制视频，不需要实现瞄准，也不需要真实相机或云台。

## 任务

编写程序，在视频中检测装甲板，并估计四个角点的三维坐标。
使用提供的模型和相机标定，每帧导出一条 JSON 记录。

语言不限，推荐 Python、C++ 或 Rust。用 CPU 即可运行，没有实时性要求，也不需要训练模型。
允许使用推理运行时、视频解码器和 OpenCV 的 PnP 函数。
你需要自己完成预处理、运行推理、解码预测结果、抑制重复检测和输出。
可以参考已有实现或使用 AI 工具，但需要能够解释自己的程序。

## 提供的数据

| 文件 | 用途 |
| --- | --- |
| [assets/0526.onnx](assets/0526.onnx) | RobotPilots 预训练装甲板检测模型 |
| [assets/small.mp4](assets/small.mp4) | 包含小装甲板的比赛片段 |
| [assets/large.mp4](assets/large.mp4) | 包含大装甲板的比赛片段 |
| [assets/calibration.yaml](assets/calibration.yaml) | 两个片段共用的相机内参和畸变参数 |

每段视频长 12 秒，共 360 帧，30 FPS，分辨率 1440 × 1080，画面没有叠加标注。
用同一个程序分别处理两段视频。文件名只是提示其中的目标，不能根据文件名决定装甲板大小。
检测所有颜色，包括友方和未点亮的装甲板。有些帧可能没有可用检测。

相机在运动。三维输出使用每帧的相机坐标系，单位为米：**+X 向右、+Y 向下、+Z 向前**。
不要求输出场地/世界坐标，也不要求跨帧跟踪。

## 需要完成的流程

1. 按顺序读取视频帧。
2. 预处理图像，运行模型推理。
3. 解码装甲板角点、置信度、颜色和图案，抑制重复检测。
4. 将角点恢复到原始图像坐标。
5. 选择大小装甲板几何模型，用 PnP 估计位姿。
6. 将四个物理角点变换到相机坐标系，写出 JSON Lines。

程序需要能接收模型、视频、标定和输出路径等参数，例如：

```sh
your-program --model assets/0526.onnx --video assets/small.mp4 \
  --calibration assets/calibration.yaml --output small.jsonl
```

把 `your-program` 换成实际运行命令。

## 模型提示

模型来自 [RobotPilots / RobotDetectionModel][model-repo]。
[OpenVINO 参考实现][inference-reference]展示了推理和后处理，请先读懂，再把流程整合进自己的程序。
导出模型的接口如下：

| 张量 | 形状 | 类型 |
| --- | --- | --- |
| 输入 `images` | `[1, 3, 640, 640]` | float16 |
| 输出 `output` | `[1, 25200, 22]` | float32 |

输入为 RGB 图像，数值归一化到 `[0, 1]`，通道在前。缩放时保持长宽比，
可以采用居中 letterbox（补边），填充值为 114。保留缩放比例和填充量，
之后用于逆变换。送入模型的数据类型必须与模型声明的 float16 一致。

每个输出行包含：

| 索引 | 含义 |
| --- | --- |
| `0:8` | 模型输入像素坐标下四个 `(x, y)` 角点 |
| `8` | Objectness logit |
| `9:13` | 颜色 logits |
| `13:22` | 图案 logits |

角点坐标已经解码，顺序沿装甲板自身方向为**左上、左下、右下、右上**。
保持它们与物理角点的对应关系；装甲板在图像中旋转后，不要按 x 或 y 坐标重新排序。

先用 sigmoid 处理 objectness 再筛选；颜色和图案分别取分数最高的类别。
本题模型的类别索引如下：

| 颜色索引 | JSON 值 |
| --- | --- |
| 0 | `blue` |
| 1 | `red` |
| 2 | `neutral` |
| 3 | `purple` |

| 图案索引 | JSON 值 | 本题使用的几何尺寸 |
| --- | --- | --- |
| 0 | `sentry` | 小装甲板 |
| 1 | `1` | 大装甲板 |
| 2 | `2` | 小装甲板 |
| 3 | `3` | 小装甲板 |
| 4 | `4` | 小装甲板 |
| 5 | `5` | 小装甲板 |
| 6 | `outpost` | 小装甲板 |
| 7 | `base_small` | 小装甲板 |
| 8 | `base_big` | 大装甲板 |

上游 README 对颜色顺序的描述不同；本题以这里的表格为准。
输出中的 `confidence` 就是 sigmoid(objectness)，不要再乘颜色或图案分数。
可以先用置信度阈值 0.5 和 NMS IoU 阈值 0.5，再自行调整并说明理由。
NMS 使用包围四个角点的轴对齐矩形即可；模型没有独立的检测框输出。

## PnP 提示

角点对应两条灯条的端点，不是外壳或数字图案的边缘。本题使用：

| 尺寸 | 宽 | 高 |
| --- | --- | --- |
| 小装甲板 | 0.135 m | 0.056 m |
| 大装甲板 | 0.230 m | 0.056 m |

以装甲板中心为原点，+X 向右、+Y 向下、+Z 指向板内。
宽为 `w`、高为 `h` 时，按约定顺序定义物理角点：

```text
upper-left:  (-w/2, -h/2, 0)
lower-left:  (-w/2, +h/2, 0)
lower-right: (+w/2, +h/2, 0)
upper-right: (+w/2, -h/2, 0)
```

用原图像素坐标，配合提供的相机矩阵和畸变参数。如果先对点去畸变，就使用对应的新相机矩阵，
不要再施加一次畸变。推理阶段的缩放不应改变恢复原图坐标后所用的标定。

[OpenCV PnP 文档][pnp-docs]解释了物体到相机的变换。
PnP 返回旋转和平移，两者都要作用到每个物理角点，得到 `corners_camera_m`。
平移向量只是装甲板原点的位置，不能直接作为所有角点的坐标。

四个角点共面，可以从 `SOLVEPNP_IPPE` 开始。
`IPPE_SQUARE` 假设正方形，不适用于本题的长方形。
平面 PnP 可能产生两个候选解；检查所选位姿位于相机前方，且能重投影到观测角点附近。
跟踪和基于时间连续性的消歧是可选扩展。

## 输出格式

输出 UTF-8 JSON Lines，每行一个完整 JSON 对象，外面不包数组。
从索引 0 开始按顺序输出所有帧。片段为固定帧率，
`timestamp_s = frame_index / 30`，每个片段的时间从零开始。

每帧包含 `frame_index`、`timestamp_s` 和 `detections`；每个检测包含：

| 字段 | 要求 |
| --- | --- |
| `color` | 颜色表中的字符串 |
| `pattern` | 图案表中的字符串，数字 ID 也写成字符串 |
| `confidence` | sigmoid(objectness)，范围为 0 到 1 |
| `corners_px` | 原图像素坐标下四个 `[x, y]` |
| `corners_camera_m` | 相机坐标系下四个 `[x, y, z]`，PnP 失败时为 `null` |
| `reprojection_error_px` | 角点重投影欧氏误差的 RMS，PnP 失败时为 `null` |

两个角点数组都采用左上、左下、右下、右上顺序。
重投影误差定义为四点的 `sqrt(sum(||projected_i - observed_i||²) / 4)`。
位姿失败时保留检测，将两个位姿相关字段置为 `null`，不要输出 NaN 或 Infinity。
没有检测的帧示例：

```json
{"frame_index": 0, "timestamp_s": 0.0, "detections": []}
```

不要硬编码这些视频中的检测结果、帧范围、颜色或装甲板尺寸。

[model-repo]: https://github.com/broalantaps/RobotDetectionModel
[inference-reference]: https://github.com/broalantaps/RobotDetectionModel/blob/babaebd6c8b3aeceda0ca924772b2d70d51801b4/OpenvinoInfer.cpp
[pnp-docs]: https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html
