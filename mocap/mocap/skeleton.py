"""COCO-17 body keypoints, the layout YOLO pose models output."""
JOINTS = [
    "nose", "left_eye", "right_eye", "left_ear", "right_ear",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist",
    "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle",
]
INDEX = {name: i for i, name in enumerate(JOINTS)}

# Rigid segments: their 3D length must stay (nearly) constant over a recording, which is the
# accuracy check of the whole chain.
BONES = [
    ("left_shoulder", "left_elbow"), ("left_elbow", "left_wrist"),
    ("right_shoulder", "right_elbow"), ("right_elbow", "right_wrist"),
    ("left_hip", "left_knee"), ("left_knee", "left_ankle"),
    ("right_hip", "right_knee"), ("right_knee", "right_ankle"),
    ("left_shoulder", "right_shoulder"), ("left_hip", "right_hip"),
    ("left_shoulder", "left_hip"), ("right_shoulder", "right_hip"),
]
BONE_INDEX = [(INDEX[a], INDEX[b]) for a, b in BONES]

# Drawing only (adds the face).
EDGES = BONE_INDEX + [(INDEX[a], INDEX[b]) for a, b in [
    ("nose", "left_eye"), ("nose", "right_eye"), ("left_eye", "left_ear"), ("right_eye", "right_ear"),
]]
