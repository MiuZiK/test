import pytest
from optimize_stitching.exceptions import (
    StitcherError, VideoReadError, CorruptFrameError,
    InvalidFrameError, BlackFrameError, WhiteFrameError,
    SeamDetectionError, BlendError, OutputWriteError
)


class TestStitcherExceptions:
    def test_stitcher_error_is_exception(self):
        assert issubclass(StitcherError, Exception)

    def test_video_read_error_inherits_from_stitcher_error(self):
        assert issubclass(VideoReadError, StitcherError)

    def test_corrupt_frame_error_inherits_from_video_read_error(self):
        assert issubclass(CorruptFrameError, VideoReadError)
        assert issubclass(CorruptFrameError, StitcherError)

    def test_invalid_frame_error_inherits_from_stitcher_error(self):
        assert issubclass(InvalidFrameError, StitcherError)

    def test_black_frame_error_inherits_from_invalid_frame_error(self):
        assert issubclass(BlackFrameError, InvalidFrameError)
        assert issubclass(BlackFrameError, StitcherError)

    def test_white_frame_error_inherits_from_invalid_frame_error(self):
        assert issubclass(WhiteFrameError, InvalidFrameError)
        assert issubclass(WhiteFrameError, StitcherError)

    def test_seam_detection_error_inherits_from_stitcher_error(self):
        assert issubclass(SeamDetectionError, StitcherError)

    def test_blend_error_inherits_from_stitcher_error(self):
        assert issubclass(BlendError, StitcherError)

    def test_output_write_error_inherits_from_stitcher_error(self):
        assert issubclass(OutputWriteError, StitcherError)

    def test_exception_message(self):
        msg = "test error message"
        exc = StitcherError(msg)
        assert str(exc) == msg

    def test_exception_chaining(self):
        inner = ValueError("inner")
        exc = StitcherError("outer")
        exc.__context__ = inner
        assert exc.__context__ is inner

    def test_raise_stitcher_error(self):
        with pytest.raises(StitcherError):
            raise StitcherError("test")

    def test_raise_video_read_error(self):
        with pytest.raises(VideoReadError):
            raise VideoReadError("cannot read video")

    def test_raise_corrupt_frame_error(self):
        with pytest.raises(CorruptFrameError):
            raise CorruptFrameError("frame is corrupt")

    def test_raise_black_frame_error(self):
        with pytest.raises(BlackFrameError):
            raise BlackFrameError("frame is all black")

    def test_raise_white_frame_error(self):
        with pytest.raises(WhiteFrameError):
            raise WhiteFrameError("frame is all white")

    def test_raise_seam_detection_error(self):
        with pytest.raises(SeamDetectionError):
            raise SeamDetectionError("cannot find seam")

    def test_raise_blend_error(self):
        with pytest.raises(BlendError):
            raise BlendError("blending failed")

    def test_raise_output_write_error(self):
        with pytest.raises(OutputWriteError):
            raise OutputWriteError("cannot write output")

    def test_catch_specific_exception(self):
        try:
            raise BlackFrameError("test")
        except BlackFrameError as e:
            assert str(e) == "test"
        except InvalidFrameError:
            pytest.fail("Should have caught BlackFrameError")

    def test_catch_generic_exception(self):
        try:
            raise BlackFrameError("test")
        except InvalidFrameError as e:
            assert isinstance(e, BlackFrameError)
        except StitcherError:
            pytest.fail("Should have caught InvalidFrameError")

    def test_exception_hierarchy(self):
        exceptions = [
            StitcherError,
            VideoReadError,
            CorruptFrameError,
            InvalidFrameError,
            BlackFrameError,
            WhiteFrameError,
            SeamDetectionError,
            BlendError,
            OutputWriteError,
        ]
        for exc in exceptions:
            assert issubclass(exc, StitcherError)
