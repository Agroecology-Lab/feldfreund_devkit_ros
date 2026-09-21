import unittest
from datetime import UTC, datetime

from devkit_ui.time_utils import now_utc, now_utc_str, parse_ts


class TestTimeUtils(unittest.TestCase):
    def test_now_utc_is_timezone_aware_utc(self) -> None:
        self.assertEqual(now_utc().tzinfo, UTC)

    def test_now_utc_str_round_trips_through_parse_ts(self) -> None:
        parsed = parse_ts(now_utc_str())

        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(parsed.tzinfo, UTC)

    def test_parse_ts_reads_iso_format(self) -> None:
        self.assertEqual(parse_ts('2025-04-01T09:30:00Z'), datetime(2025, 4, 1, 9, 30, 0, tzinfo=UTC))

    def test_parse_ts_reads_legacy_format(self) -> None:
        self.assertEqual(parse_ts('01-04-2025_09-30-00'), datetime(2025, 4, 1, 9, 30, 0, tzinfo=UTC))

    def test_parse_ts_returns_none_for_missing_input(self) -> None:
        self.assertIsNone(parse_ts(None))
        self.assertIsNone(parse_ts(''))

    def test_parse_ts_returns_none_for_malformed_input(self) -> None:
        self.assertIsNone(parse_ts('garbage'))
        self.assertIsNone(parse_ts('2025-13-45T99:99:99Z'))


if __name__ == '__main__':
    unittest.main()
