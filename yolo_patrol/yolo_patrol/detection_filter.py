from collections import Counter, defaultdict, deque
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional


def box_area(box: Iterable[int]) -> int:
    values = list(box or [])
    if len(values) < 4:
        return 0
    x1, y1, x2, y2 = values[:4]
    return max(0, int(x2) - int(x1)) * max(0, int(y2) - int(y1))


@dataclass
class ObjectCandidate:
    class_name: str
    score: float
    area: int
    box: List[int]
    timestamp: float


@dataclass
class DetectionSummary:
    status: str
    confirmed_class: str = ''
    count: int = 0
    avg_score: float = 0.0
    max_score: float = 0.0
    max_area: int = 0
    message_count: int = 0
    valid_count: int = 0
    detail: str = ''

    @property
    def confirmed(self):
        return self.status == 'confirmed' and bool(self.confirmed_class)


class DetectionWindow:
    """Collects YOLO ObjectsInfo messages during one inspection window."""

    def __init__(self, watch_classes, min_score=0.6, stable_min_count=3):
        self.watch_classes = set(watch_classes or [])
        self.min_score = float(min_score)
        self.stable_min_count = int(stable_min_count)
        self.message_count = 0
        self.candidates: List[ObjectCandidate] = []
        self.rejected_low_score = 0
        self.rejected_unwatched = 0

    def add_objects(self, objects, timestamp):
        self.message_count += 1
        for obj in objects:
            class_name = str(getattr(obj, 'class_name', '')).strip()
            score = float(getattr(obj, 'score', 0.0))
            if self.watch_classes and class_name not in self.watch_classes:
                self.rejected_unwatched += 1
                continue
            if score < self.min_score:
                self.rejected_low_score += 1
                continue
            box = list(getattr(obj, 'box', []) or [])
            self.candidates.append(
                ObjectCandidate(
                    class_name=class_name,
                    score=score,
                    area=box_area(box),
                    box=box,
                    timestamp=timestamp,
                )
            )

    def summarize(self):
        if self.message_count == 0:
            return DetectionSummary(
                status='no_yolo_data',
                message_count=0,
                valid_count=0,
                detail='no /yolo/object_detect messages during observe window',
            )

        if not self.candidates:
            return DetectionSummary(
                status='no_valid_object',
                message_count=self.message_count,
                valid_count=0,
                detail=(
                    'YOLO messages received, but no watched class met '
                    f'min_score={self.min_score:.2f}; '
                    f'low_score={self.rejected_low_score}, '
                    f'unwatched={self.rejected_unwatched}'
                ),
            )

        grouped: Dict[str, List[ObjectCandidate]] = defaultdict(list)
        for candidate in self.candidates:
            grouped[candidate.class_name].append(candidate)

        ranked = sorted(
            grouped.items(),
            key=lambda item: (
                len(item[1]),
                sum(c.score for c in item[1]) / len(item[1]),
                max(c.area for c in item[1]),
            ),
            reverse=True,
        )
        class_name, candidates = ranked[0]
        count = len(candidates)
        avg_score = sum(c.score for c in candidates) / count
        max_score = max(c.score for c in candidates)
        max_area = max(c.area for c in candidates)
        status = 'confirmed' if count >= self.stable_min_count else 'unstable'
        detail = (
            f'class={class_name}, count={count}, avg_score={avg_score:.2f}, '
            f'max_score={max_score:.2f}, max_area={max_area}'
        )
        return DetectionSummary(
            status=status,
            confirmed_class=class_name if status == 'confirmed' else '',
            count=count,
            avg_score=avg_score,
            max_score=max_score,
            max_area=max_area,
            message_count=self.message_count,
            valid_count=len(self.candidates),
            detail=detail,
        )


class MovingWatchBuffer:
    """Keeps lightweight candidates seen while the robot is navigating."""

    def __init__(self, watch_classes, min_score=0.7, max_age_sec=15.0):
        self.watch_classes = set(watch_classes or [])
        self.min_score = float(min_score)
        self.max_age_sec = float(max_age_sec)
        self.candidates = deque(maxlen=100)

    def add_objects(self, objects, timestamp):
        for obj in objects:
            class_name = str(getattr(obj, 'class_name', '')).strip()
            score = float(getattr(obj, 'score', 0.0))
            if self.watch_classes and class_name not in self.watch_classes:
                continue
            if score < self.min_score:
                continue
            box = list(getattr(obj, 'box', []) or [])
            self.candidates.append(
                ObjectCandidate(
                    class_name=class_name,
                    score=score,
                    area=box_area(box),
                    box=box,
                    timestamp=timestamp,
                )
            )
        self.prune(timestamp)

    def prune(self, now):
        while self.candidates and now - self.candidates[0].timestamp > self.max_age_sec:
            self.candidates.popleft()

    def recent_summary(self, now) -> Optional[str]:
        self.prune(now)
        if not self.candidates:
            return None
        counts = Counter(c.class_name for c in self.candidates)
        class_name, count = counts.most_common(1)[0]
        best_score = max(c.score for c in self.candidates if c.class_name == class_name)
        return f'{class_name} x{count}, best_score={best_score:.2f}'
