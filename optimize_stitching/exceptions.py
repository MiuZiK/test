class StitcherError(Exception):
    pass


class VideoReadError(StitcherError):
    pass


class CorruptFrameError(VideoReadError):
    pass


class InvalidFrameError(StitcherError):
    pass


class BlackFrameError(InvalidFrameError):
    pass


class WhiteFrameError(InvalidFrameError):
    pass


class SeamDetectionError(StitcherError):
    pass


class BlendError(StitcherError):
    pass


class OutputWriteError(StitcherError):
    pass
