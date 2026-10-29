"""Vision failures are SequenceErrors: they stop the job without retries or recovery moves."""
from ..controller import SequenceError


class VisionError(SequenceError):
    pass


class CalibrationError(VisionError):
    pass


class QualityGateError(VisionError):
    """An observation was rejected; ``reasons`` lists each failed gate with its values."""

    def __init__(self, message, reasons=(), report=None):
        self.reasons = list(reasons)
        self.report = report or {}
        detail = '; '.join(self.reasons)
        super().__init__(f'{message}: {detail}' if detail else message)


class TargetValidationError(VisionError):
    """A computed pick/place pose failed a safety or plausibility check."""

    def __init__(self, message, reasons=()):
        self.reasons = list(reasons)
        detail = '; '.join(self.reasons)
        super().__init__(f'{message}: {detail}' if detail else message)
