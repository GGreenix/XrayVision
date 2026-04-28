from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from xray_core.math_utils import clamp, vector_distance
from xray_interfaces.msg import TrackedObject, TrackedObjectArray


@dataclass
class TrackState:
    track_id: str
    class_id: str
    position: list[float]
    confidence: float
    dimensions: list[float]
    covariance: list[float]
    last_seen_sec: float
    source_frame: str


class WorldModel(Node):
    def __init__(self) -> None:
        super().__init__("world_model")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("match_radius_m", 3.0)
        self.declare_parameter("stale_after_s", 10.0)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("smoothing_alpha", 0.65)

        self.world_frame = str(self.get_parameter("world_frame").value)
        self.match_radius_m = float(self.get_parameter("match_radius_m").value)
        self.stale_after_s = float(self.get_parameter("stale_after_s").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.smoothing_alpha = clamp(float(self.get_parameter("smoothing_alpha").value), 0.0, 1.0)

        self.tracks: dict[str, TrackState] = {}
        self.next_id = 1
        self.publisher = self.create_publisher(TrackedObjectArray, "/xray/world/objects", 10)
        self.create_subscription(TrackedObjectArray, "/xray/perception/objects_raw", self.objects_callback, 20)
        self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self.publish_tracks)

    def objects_callback(self, message: TrackedObjectArray) -> None:
        now_sec = self._clock_seconds()
        self._prune_stale(now_sec)
        for tracked_object in message.objects:
            position = [
                float(tracked_object.pose.position.x),
                float(tracked_object.pose.position.y),
                float(tracked_object.pose.position.z),
            ]
            matched_track_id = self._find_match(tracked_object.class_id, position)
            if matched_track_id is None:
                self._create_track(tracked_object, position, now_sec)
            else:
                self._update_track(matched_track_id, tracked_object, position, now_sec)
        self.publish_tracks()

    def publish_tracks(self) -> None:
        now_sec = self._clock_seconds()
        self._prune_stale(now_sec)

        message = TrackedObjectArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.world_frame

        for track_id in sorted(self.tracks.keys()):
            state = self.tracks[track_id]
            tracked_object = TrackedObject()
            tracked_object.tracking_id = state.track_id
            tracked_object.class_id = state.class_id
            tracked_object.confidence = float(state.confidence)
            tracked_object.pose.position.x = float(state.position[0])
            tracked_object.pose.position.y = float(state.position[1])
            tracked_object.pose.position.z = float(state.position[2])
            tracked_object.pose.orientation.w = 1.0
            tracked_object.dimensions.x = float(state.dimensions[0])
            tracked_object.dimensions.y = float(state.dimensions[1])
            tracked_object.dimensions.z = float(state.dimensions[2])
            tracked_object.position_covariance = [float(value) for value in state.covariance]
            tracked_object.last_observed = self._seconds_to_time(state.last_seen_sec)
            tracked_object.is_persistent = True
            tracked_object.source_frame = state.source_frame
            message.objects.append(tracked_object)

        self.publisher.publish(message)

    def _create_track(self, tracked_object: TrackedObject, position, now_sec: float) -> None:
        track_id = f"obj-{self.next_id:04d}"
        self.next_id += 1
        self.tracks[track_id] = TrackState(
            track_id=track_id,
            class_id=tracked_object.class_id,
            position=list(position),
            confidence=float(tracked_object.confidence),
            dimensions=[
                float(tracked_object.dimensions.x),
                float(tracked_object.dimensions.y),
                float(tracked_object.dimensions.z),
            ],
            covariance=[float(value) for value in tracked_object.position_covariance],
            last_seen_sec=now_sec,
            source_frame=tracked_object.source_frame,
        )

    def _update_track(self, track_id: str, tracked_object: TrackedObject, position, now_sec: float) -> None:
        state = self.tracks[track_id]
        alpha = self.smoothing_alpha
        state.position = [
            alpha * position[index] + (1.0 - alpha) * state.position[index] for index in range(3)
        ]
        new_dimensions = [
            float(tracked_object.dimensions.x),
            float(tracked_object.dimensions.y),
            float(tracked_object.dimensions.z),
        ]
        state.dimensions = [
            alpha * new_dimensions[index] + (1.0 - alpha) * state.dimensions[index] for index in range(3)
        ]
        state.confidence = alpha * float(tracked_object.confidence) + (1.0 - alpha) * state.confidence
        state.covariance = [float(value) for value in tracked_object.position_covariance]
        state.last_seen_sec = now_sec
        state.source_frame = tracked_object.source_frame

    def _find_match(self, class_id: str, position) -> str | None:
        best_track_id = None
        best_distance = self.match_radius_m
        for track_id, state in self.tracks.items():
            if state.class_id != class_id:
                continue
            distance = vector_distance(state.position, position)
            if distance <= best_distance:
                best_distance = distance
                best_track_id = track_id
        return best_track_id

    def _prune_stale(self, now_sec: float) -> None:
        stale_ids = [
            track_id
            for track_id, state in self.tracks.items()
            if now_sec - state.last_seen_sec > self.stale_after_s
        ]
        for track_id in stale_ids:
            del self.tracks[track_id]

    def _clock_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _seconds_to_time(self, seconds: float):
        whole_seconds = int(seconds)
        nanoseconds = int((seconds - whole_seconds) * 1e9)
        message = self.get_clock().now().to_msg()
        message.sec = whole_seconds
        message.nanosec = nanoseconds
        return message


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorldModel()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
