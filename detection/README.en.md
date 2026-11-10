# Armor detection and pose estimation

[中文](README.md) · [English](README.en.md)

This is one of two optional paths. **Choose detection or aiming; you do not need to do both.**
See the [main README](../README.en.md) for the gimbal integration scenario.
This path uses recorded video and needs no aiming implementation, physical camera, or gimbal.

Write a program that detects armor plates in a video and estimates their four corners in 3D.
Use the supplied model and camera calibration. Export one JSON record for every frame.

You may use any language. We recommend Python, C++, or Rust. CPU execution is sufficient;
there is no real-time requirement and no model training. Inference runtimes, video decoders,
and OpenCV's PnP functions are allowed. Write your own preprocessing, inference integration,
prediction decoding, duplicate suppression, and output code. You may consult reference
implementations and use AI tools, but you should be able to explain your program.

## Supplied data

| File | Purpose |
| --- | --- |
| [assets/0526.onnx](assets/0526.onnx) | Pretrained RobotPilots armor detector |
| [assets/small.mp4](assets/small.mp4) | Short competition excerpt containing small armor |
| [assets/large.mp4](assets/large.mp4) | Short competition excerpt containing large armor |
| [assets/calibration.yaml](assets/calibration.yaml) | Camera intrinsics and distortion for both clips |

Each clip is 12 seconds, 360 frames, 30 FPS, and 1440 by 1080 pixels. They have no overlays.
Process both clips separately with the same program. The names describe useful targets in
each clip; do not use the filename to select armor size. Detect all colors, including friendly
and inactive armor. Some frames may contain no usable detections.

The camera moves. Output coordinates relative to the camera at each frame, in metres:
**+X right, +Y down, +Z forward**. Field/world coordinates and tracking across frames are not
required.

## What to implement

1. Read the video frames in order.
2. Prepare each frame for the model and run inference.
3. Decode armor corners, confidence, color, and pattern. Suppress duplicate predictions.
4. Restore corner coordinates to the original image resolution.
5. Select small or large armor geometry and estimate its pose using PnP.
6. Transform the four physical corners into camera coordinates and write JSON Lines.

Provide a command that accepts the model, video, calibration, and output paths. For example,
the interface could look like this; replace `your-program` with your actual command:

```sh
your-program --model assets/0526.onnx --video assets/small.mp4 \
  --calibration assets/calibration.yaml --output small.jsonl
```

## Model hints

The model comes from [RobotPilots / RobotDetectionModel][model-repo]. Its
[OpenVINO reference][inference-reference] shows inference and postprocessing. Study the
reference, then integrate the steps into your own program.

The supplied export has the following tensor contract:

| Tensor | Shape | Type |
| --- | --- | --- |
| Input `images` | `[1, 3, 640, 640]` | float16 |
| Output `output` | `[1, 25200, 22]` | float32 |

Prepare RGB input with values in `[0, 1]` and channels first. Preserve aspect ratio when
resizing; centered letterboxing with padding value 114 is a suitable approach. Keep the resize
scale and padding so you can invert that transform later. Feed the model its declared dtype.

Each output row contains:

| Indices | Meaning |
| --- | --- |
| `0:8` | Four `(x, y)` corners in model-input pixels |
| `8` | Objectness logit |
| `9:13` | Color logits |
| `13:22` | Pattern logits |

The corners are already decoded coordinates. Their order is **upper-left, lower-left,
lower-right, upper-right**, following the plate's local orientation. Preserve the association
with the physical corners; do not sort them by image x or y after rotation.

Apply sigmoid to objectness before thresholding. Use the largest color and pattern scores
to classify each retained prediction. For the supplied model, use these index mappings:

| Color index | JSON value |
| --- | --- |
| 0 | `blue` |
| 1 | `red` |
| 2 | `neutral` |
| 3 | `purple` |

| Pattern index | JSON value | Geometry for this assignment |
| --- | --- | --- |
| 0 | `sentry` | Small |
| 1 | `1` | Large |
| 2 | `2` | Small |
| 3 | `3` | Small |
| 4 | `4` | Small |
| 5 | `5` | Small |
| 6 | `outpost` | Small |
| 7 | `base_small` | Small |
| 8 | `base_big` | Large |

The upstream README describes colors differently; the table above is the contract for the
supplied weights. `confidence` in your output is sigmoid(objectness), without multiplying it
by a color or pattern score. Start with a confidence threshold of 0.5 and NMS IoU threshold
of 0.5. You may tune these; document your choices. An axis-aligned box enclosing the four
corners is sufficient for NMS. There is no separate bounding-box output in this model.

## PnP hints

The points represent the ends of the two lightbars, not the outer housing or printed number.
Use these dimensions for the assignment:

| Size | Width | Height |
| --- | --- | --- |
| Small | 0.135 m | 0.056 m |
| Large | 0.230 m | 0.056 m |

Define the plate origin at its center, +X to its right, +Y down, and +Z into the plate.
For width `w` and height `h`, the physical points in the required order are:

```text
upper-left:  (-w/2, -h/2, 0)
lower-left:  (-w/2, +h/2, 0)
lower-right: (+w/2, +h/2, 0)
upper-right: (+w/2, -h/2, 0)
```

Use the original-image pixel coordinates with the supplied camera matrix and distortion.
If you undistort points first, use the camera matrix for those points and avoid applying
distortion a second time. Resizing for inference must not change the calibration used for
the restored image coordinates.

[OpenCV's PnP documentation][pnp-docs] explains the object-to-camera transform. PnP estimates
rotation and translation; apply both to each physical corner to obtain `corners_camera_m`.
The translation vector alone is the plate origin, not all four corners.

The points are coplanar. `SOLVEPNP_IPPE` is a useful starting point; `IPPE_SQUARE` assumes a
square and does not match these plates. Planar pose estimation can produce two candidates.
Check that the selected pose lies in front of the camera and reprojects near the observed
corners. Tracking or temporal disambiguation is optional.

## Output

Write UTF-8 JSON Lines: one complete JSON object per line, without a surrounding array.
Include every frame, starting at index 0, in order. For these constant-rate clips,
`timestamp_s = frame_index / 30`; time starts at zero in each excerpt.

Each frame has `frame_index`, `timestamp_s`, and `detections`. Each detection has:

| Field | Required value |
| --- | --- |
| `color` | A string from the color table |
| `pattern` | A string from the pattern table, including numeric IDs as strings |
| `confidence` | Sigmoid(objectness), between 0 and 1 |
| `corners_px` | Four `[x, y]` pairs in original-image pixels |
| `corners_camera_m` | Four `[x, y, z]` triples in camera coordinates, or `null` if PnP fails |
| `reprojection_error_px` | RMS Euclidean corner reprojection error, or `null` if PnP fails |

Both corner arrays use the same upper-left, lower-left, lower-right, upper-right order.
Reprojection error means `sqrt(sum(||projected_i - observed_i||²) / 4)` over the four corners.
Keep a detection with a failed pose and set both pose fields to `null`; never emit NaN or
Infinity. A frame without detections looks like this:

```json
{"frame_index": 0, "timestamp_s": 0.0, "detections": []}
```

Do not hard-code detections, frame ranges, colors, or armor sizes from these videos.

[model-repo]: https://github.com/broalantaps/RobotDetectionModel
[inference-reference]: https://github.com/broalantaps/RobotDetectionModel/blob/babaebd6c8b3aeceda0ca924772b2d70d51801b4/OpenvinoInfer.cpp
[pnp-docs]: https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html
