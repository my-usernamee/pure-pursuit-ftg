import numpy as np


class FTGLogic:
    def __init__(self):
        self.max_speed = 6.0
        self.max_steering = 0.40
        self.car_width = 0.50
        self.prev_steering = 0.0

    def get_disparities(self, ranges, angle_increment):
        proc_ranges = ranges.copy()
        for i in range(1, len(ranges)):
            if abs(ranges[i] - ranges[i - 1]) > 0.5:
                closer_idx = i if ranges[i] < ranges[i - 1] else i - 1
                further_idx = i - 1 if ranges[i] < ranges[i - 1] else i

                dist = ranges[closer_idx]
                if dist <= 0:
                    continue

                angle = np.arcsin(min(1.0, self.car_width / dist))
                idx_span = int(angle / angle_increment)

                if closer_idx == i:
                    start = max(0, further_idx - idx_span)
                    proc_ranges[start:further_idx + 1] = dist
                else:
                    end = min(len(ranges) - 1, further_idx + idx_span)
                    proc_ranges[further_idx:end + 1] = dist
        return proc_ranges

    def _extract_gaps(self, safe_ranges, min_clearance):
        masked = np.where(safe_ranges > min_clearance, 1, 0)
        slices = np.split(np.arange(len(masked)), np.where(np.diff(masked) != 0)[0] + 1)
        return [s for s in slices if len(s) > 0 and masked[s[0]] == 1]

    def _score_candidate(self, safe_ranges, idx, gap, angle_min, angle_increment, start, preferred_steer):
        width = len(gap)
        path_dist = float(safe_ranges[idx])

        c_left = max(0, idx - 10)
        c_right = min(len(safe_ranges) - 1, idx + 10)
        corridor = safe_ranges[c_left:c_right + 1]
        corridor_mean = float(np.mean(corridor))
        corridor_min = float(np.min(corridor))

        global_idx = idx + start
        steer = angle_min + global_idx * angle_increment
        steer = float(np.clip(steer, -self.max_steering, self.max_steering))

        width_norm = width / (len(safe_ranges) + 1e-6)
        align = 1.0 - min(abs(steer - preferred_steer) / (self.max_steering + 1e-6), 1.0)

        score = (
            2.2 * path_dist +
            1.4 * corridor_min +
            0.8 * corridor_mean +
            0.8 * width_norm +
            0.7 * align -
            0.3 * abs(steer)
        )

        return {
            "steer": steer,
            "score": score,
            "path_dist": path_dist,
            "corridor_min": corridor_min,
            "min_forward": float(np.min(safe_ranges)),
        }

    def process_lidar(self, msg, preferred_steer=0.0):
        ranges = np.array(msg.ranges)
        ranges[np.isnan(ranges)] = 0.0
        ranges[np.isinf(ranges)] = msg.range_max if msg.range_max > 0.0 else 10.0

        center = len(ranges) // 2
        fov = int(75 / (msg.angle_increment * 180 / np.pi))
        start, end = center - fov, center + fov
        forward_ranges = ranges[start:end].copy()

        safe_ranges = self.get_disparities(forward_ranges, msg.angle_increment)
        mid = len(safe_ranges) // 2

        left = safe_ranges[mid:] if mid < len(safe_ranges) else safe_ranges
        right = safe_ranges[:mid] if mid > 0 else safe_ranges
        left_open = float(np.percentile(left, 80)) if len(left) else 0.0
        right_open = float(np.percentile(right, 80)) if len(right) else 0.0

        min_forward = float(np.min(safe_ranges)) if len(safe_ranges) else 0.0

        side_hint = None
        if min_forward < 1.2:
            if left_open - right_open > 0.3:
                side_hint = "left"
            elif right_open - left_open > 0.3:
                side_hint = "right"
            else:
                side_hint = "left" if preferred_steer >= 0.0 else "right"

        gaps = self._extract_gaps(safe_ranges, min_clearance=1.2)
        if not gaps:
            gaps = self._extract_gaps(safe_ranges, min_clearance=1.0)
        if not gaps:
            gaps = self._extract_gaps(safe_ranges, min_clearance=0.8)

        if not gaps:
            left_min = float(np.min(left)) if len(left) else float(np.min(safe_ranges))
            right_min = float(np.min(right)) if len(right) else float(np.min(safe_ranges))
            avoid_dir = -1.0 if left_min < right_min else 1.0
            steer = 0.25 * avoid_dir
            self.prev_steering = 0.6 * self.prev_steering + 0.4 * steer
            return 1.0, float(self.prev_steering)

        def build_candidates(force_side):
            candidates = []
            for g in gaps:
                interior = np.arange(g[0], g[-1] + 1)
                if force_side == "left":
                    interior = interior[interior >= mid]
                elif force_side == "right":
                    interior = interior[interior < mid]
                if len(interior) == 0:
                    continue
                idx = int(interior[np.argmax(safe_ranges[interior])])
                candidates.append(self._score_candidate(safe_ranges, idx, g, msg.angle_min, msg.angle_increment, start, preferred_steer))
            return candidates

        candidates = build_candidates(side_hint) if side_hint else build_candidates(None)
        if not candidates:
            candidates = build_candidates(None)

        best = max(candidates, key=lambda x: x["score"])
        target_steer = best["steer"]

        if min_forward < 1.0:
            self.prev_steering = 0.3 * self.prev_steering + 0.7 * target_steer
        else:
            self.prev_steering = 0.6 * self.prev_steering + 0.4 * target_steer

        steer_ratio = abs(self.prev_steering) / self.max_steering
        path_dist = best["path_dist"]

        if min_forward < 0.7:
            speed = 1.2
        elif path_dist < 2.0:
            speed = 2.0
        elif path_dist < 3.5:
            speed = 2.8
        else:
            speed = self.max_speed * (1.0 - 0.35 * steer_ratio)
            speed = max(2.5, speed)

        return float(speed), float(self.prev_steering)
