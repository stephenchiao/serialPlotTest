"""只用于人工调试的原始模型框；没有物料映射时也可预览，不授权抓取。"""


def model_detection_summary(result):
    return tuple(
        (item.class_id, item.class_name, round(item.confidence, 3))
        for item in result.raw_model_detections
    )


def draw_model_detections(cv2, frame, result):
    for item in result.raw_model_detections:
        x1, y1, x2, y2 = (round(value) for value in item.xyxy)
        color = (160, 160, 160)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)
        cv2.putText(
            frame, f"class={item.class_id} conf={item.confidence:.2f}",
            (x1, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
        )
