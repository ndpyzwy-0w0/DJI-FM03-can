"""Offline regression checks for the paired replay startup builds.

Uses the installed Xtensa compiler to preprocess the actual source and compile
static assertions for its selected gate expression. Never opens a serial port.
Run: .venv/Scripts/python.exe -m unittest discover -s tools -p test_replay_timing.py -v
"""
import configparser
from pathlib import Path
import re
import shlex
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]
COMPILER = (Path.home() / ".platformio/packages/toolchain-xtensa-esp32s3/bin/"
            "xtensa-esp32s3-elf-g++.exe")


def settings(env):
    config = configparser.ConfigParser(interpolation=None)
    config.read(ROOT / "platformio.ini", encoding="utf-8")
    section = config["env:" + env]
    flags = section["build_flags"]
    for _ in range(10):
        if "${" not in flags:
            break
        flags = re.sub(r"\$\{([^}]+)\}",
                       lambda match: config[match[1].rsplit(".", 1)[0]][
                           match[1].rsplit(".", 1)[1]], flags)
    if "${" in flags:
        raise ValueError("unresolved build flags")
    source_filter = section.get("build_src_filter")
    if source_filter is None:
        source_filter = config[section["extends"]]["build_src_filter"]
    source = ROOT / "src" / re.search(r"\+<([^>]+)>", source_filter)[1]
    return source, [f for f in shlex.split(flags)
                    if f not in shlex.split(section.get("build_unflags", ""))]


def selected_gate(env, overrides=()):
    source, flags = settings(env)
    text = "\n".join(line for line in source.read_text(encoding="utf-8").splitlines()
                     if not line.startswith("#include"))
    result = subprocess.run([str(COMPILER), "-E", "-P", "-x", "c++", *flags, *overrides, "-"],
                            input=text, text=True, encoding="utf-8",
                            capture_output=True, check=True)
    match = re.search(r"bool steady_time = ([^;]+);", result.stdout)
    return match[1] if match else "true", result.stdout


def guards_of(output):
    """Extract the firmware's static assertions as compilable statements.

    The preprocessor prints each assertion over several lines, and the message
    strings contain ';', so the statements must be collected whole up to their
    terminating semicolon rather than up to the end of a line. The compiled probe
    duration a couple of them refer to is declared by the caller.
    """
    statements = []
    pending = None
    for line in output.splitlines():
        if pending is None:
            if line.lstrip().startswith("static_assert("):
                pending = line.strip()
        else:
            pending += " " + line.strip()
        if pending is not None and pending.endswith(";"):
            statements.append(pending)
            pending = None
    return "\n".join(statements)


class ReplayTimingTests(unittest.TestCase):
    def probe_timeline(self, env, overrides=()):
        """Return the probe window (start, duration), the assertions and the selector.

        The window is read from the compiled selector and from the firmware's own
        duration assertion, so the check follows what the build actually contains
        rather than what the flags were meant to say. An environment with no probe
        has no assertion, so its duration comes from the flags.
        """
        _, flags = settings(env)
        _, output = selected_gate(env, overrides)
        selector = re.search(r"static constexpr int16_t replay_dial_value\([^}]+\}", output)[0]
        guards = guards_of(output)
        start_match = re.search(r"elapsed >= ([0-9]+)", selector)
        start = (int(start_match.group(1)) if start_match else
                 next((int(flag.split("=", 1)[1]) for flag in flags
                       if flag.startswith("-DREPLAY_DIAL_PROBE_START_MS=")), 5000))
        if not start_match:
            # No probe configured: the selector is a constant zero and the flags are
            # the only record of the (unused) window.
            duration = next((int(flag.split("=", 1)[1]) for flag in flags
                             if flag.startswith("-DREPLAY_DIAL_PROBE_DURATION_MS=")), 200)
            return start, duration, guards, selector
        # When a probe is configured the selector switches at exactly the compiled
        # duration, so the firmware's own bounds assertion carries it.
        bound_match = re.search(
            r"Probe duration.{0,80}?([0-9]+) > 0 && ([0-9]+) <= ([0-9]+)", guards)
        if bound_match:
            duration = int(bound_match.group(3))
        else:
            duration = next((int(flag.split("=", 1)[1]) for flag in flags
                             if flag.startswith("-DREPLAY_DIAL_PROBE_DURATION_MS=")), 200)
        return start, duration, guards, selector

    def compiled_code(self, start, duration, guards, selector):
        """A translation unit holding the firmware's assertions plus the selector."""
        return (f"#include <stdint.h>\nstatic constexpr unsigned long replay_probe_duration_ms = "
                f"{duration}UL;\nstatic constexpr unsigned long replay_probe_start_ms = {start}UL;\n"
                + guards + "\n" + selector)

    def assert_rejected(self, env, overrides, message):
        """The candidate must not compile when one of its bounds is violated."""
        start, duration, guards, selector = self.probe_timeline(env, overrides)
        result = subprocess.run([str(COMPILER), "-std=c++11", "-fsyntax-only", "-x", "c++", "-"],
                                input=self.compiled_code(start, duration, guards, selector),
                                text=True, encoding="utf-8", capture_output=True)
        self.assertNotEqual(result.returncode, 0, f"{env} {overrides} compiled but must not")
        self.assertIn(message, result.stderr)

    def assert_boundaries(self, start, duration, guards, selector, value):
        """Prove the compiled selector switches at the exact compiled boundaries.

        A positive value must appear only inside [start, start + duration); zero is
        checked by matching the sign, so both signs are covered.
        """
        code = self.compiled_code(start, duration, guards, selector)
        for offset, expect_nonzero in ([-1, False], [0, True],
                                       [duration - 1, True], [duration, False],
                                       [duration + 1, False]):
            elapsed = start + offset
            if elapsed < 0:
                continue
            if expect_nonzero:
                code += (f'\nstatic_assert(replay_dial_value({elapsed}UL) == {value}, '
                         '"inside the probe window");')
            else:
                code += (f'\nstatic_assert(replay_dial_value({elapsed}UL) == 0, '
                         '"outside the probe window");')
        code += ('\nstatic_assert(replay_dial_value(0UL) == 0, "probe starts at zero");'
                 '\nstatic_assert(replay_dial_value(4294967295UL) == 0, "probe ends at zero");')
        subprocess.run([str(COMPILER), "-std=c++11", "-fsyntax-only", "-x", "c++", "-"],
                       input=code, text=True, encoding="utf-8",
                       capture_output=True, check=True)

    def test_motion_probe_boundaries_and_manual_start_guard(self):
        _, plus_flags = settings("replay_gate0_probe16_echo")
        _, minus_flags = settings("replay_gate0_probe_minus16_echo")
        trim = lambda flags: [f for f in flags if not f.startswith("-DREPLAY_DIAL_PROBE_VALUE=")]
        self.assertEqual(trim(plus_flags), trim(minus_flags))
        self.assertIn("-DREPLAY_DIAL_PROBE_VALUE=-16", minus_flags)
        self.assertNotIn("-DREPLAY_DIAL_PROBE_VALUE=16", minus_flags)
        for env, value in (("replay_gate0_probe16_echo", 16),
                           ("replay_gate0_probe_minus16_echo", -16),
                           ("replay_gate0_stable_50ms_echo", 0)):
            with self.subTest(env=env):
                start, duration, guards, selector = self.probe_timeline(env)
                self.assertEqual((start, duration), (5000, 200))
                if value:
                    self.assert_boundaries(start, duration, guards, selector, value)
                else:
                    # No probe configured: every environment must stay on zero.
                    code = ("#include <stdint.h>\n" + selector
                            + "\nstatic_assert(replay_dial_value(0UL) == 0, \"zero\");"
                              "\nstatic_assert(replay_dial_value(5000UL) == 0, \"zero\");"
                              "\nstatic_assert(replay_dial_value(4294967295UL) == 0, \"zero\");")
                    subprocess.run([str(COMPILER), "-std=c++11", "-fsyntax-only", "-x", "c++", "-"],
                                   input=code, text=True, encoding="utf-8",
                                   capture_output=True, check=True)
        self.assert_rejected("replay_gate0_probe16_echo",
                             ("-UREPLAY_AUTO_START", "-DREPLAY_AUTO_START=1"),
                             "Motion probe requires manual start")

    def test_hold_candidate_holds_one_value_then_returns_to_zero(self):
        """The prepared controllability candidate differs from the verified pulse
        build only by holding the same value for a longer, bounded time."""
        _, hold_flags = settings("replay_gate0_hold16_echo")
        _, pulse_flags = settings("replay_gate0_probe_minus16_echo")

        def trim(flags):
            # Both the redefined flags and the start value (unflagged and redefined to
            # the same value only to avoid a macro-redefinition warning) are ignored.
            return [f for f in flags
                    if not f.startswith(("-DREPLAY_DIAL_PROBE_HOLD=",
                                         "-DREPLAY_DIAL_PROBE_DURATION_MS=",
                                         "-DREPLAY_SECONDS=",
                                         "-DREPLAY_DIAL_PROBE_START_MS="))]
        self.assertEqual(sorted(trim(hold_flags)), sorted(trim(pulse_flags)))
        # Each redefined macro appears exactly once, or the compiler warns.
        for name in ("REPLAY_SECONDS", "REPLAY_DIAL_PROBE_VALUE",
                     "REPLAY_DIAL_PROBE_DURATION_MS", "REPLAY_DIAL_PROBE_START_MS"):
            with self.subTest(macro=name):
                definitions = [f for f in hold_flags if f.startswith("-D" + name + "=")]
                self.assertEqual(len(definitions), 1, definitions)
        for flag in ("-DREPLAY_DIAL_PROBE_VALUE=-16", "-DREPLAY_DIAL_PROBE_HOLD=1",
                     "-DREPLAY_DIAL_PROBE_DURATION_MS=3000",
                     "-DREPLAY_AUTO_START=0", "-DREPLAY_SECONDS=20",
                     "-DREPLAY_INTERVAL_MS=50", "-DTX_QUEUE_LEN=0",
                     "-DREPLAY_SINGLE_ROUTE=1", "-DREPLAY_SYNC_RX=1",
                     "-DREPLAY_SEQ_START=0", "-DREPLAY_SEQ_STEP=0"):
            self.assertIn(flag, hold_flags)
        # The 5 s warmup must stay at exactly the value the pulse comparison used.
        self.assertIn("-DREPLAY_DIAL_PROBE_START_MS=5000", hold_flags)

        start, duration, guards, selector = self.probe_timeline("replay_gate0_hold16_echo")
        self.assertEqual((start, duration), (5000, 3000))
        self.assert_boundaries(start, duration, guards, selector, -16)
        # The value must be selected against the live clock, not an expired pulse
        # timestamp, or a hold could be applied late or skipped entirely.
        _, output = selected_gate("replay_gate0_hold16_echo")
        self.assertIn("unsigned long dial_elapsed = millis() - started;", output)
        self.assertIn("int16_t value = replay_dial_value(dial_elapsed);", output)
        self.assertIn("replay_probe_duration_ms", output)

    def test_hold_pair_changes_only_the_value_sign(self):
        """The prepared symmetry pair must differ only by the commanded sign."""
        _, minus_flags = settings("replay_gate0_hold16_echo")
        _, plus_flags = settings("replay_gate0_holdup16_echo")

        def trim(flags):
            return [f for f in flags if not f.startswith("-DREPLAY_DIAL_PROBE_VALUE=")]
        # Holds for both environments come from the same expansion, so the flags are
        # the same apart from the value - which is read from the compiled selector
        # below rather than from the flags, because build_unflags removes inherited
        # definitions that this reader cannot see.
        self.assertEqual(sorted(trim(minus_flags)), sorted(trim(plus_flags)))
        for env, value in (("replay_gate0_hold16_echo", -16),
                           ("replay_gate0_holdup16_echo", 16)):
            with self.subTest(env=env):
                start, duration, guards, selector = self.probe_timeline(env)
                self.assertEqual((start, duration), (5000, 3000))
                compiled = int(re.search(r"\? \(int16_t\)(-?[0-9]+) :", selector).group(1))
                self.assertEqual(compiled, value)
                self.assert_boundaries(start, duration, guards, selector, value)

    def test_hold_bounds_are_enforced(self):
        """A hold must be non-zero, bounded, and return to zero before the window ends."""
        # The compiler truncates long assert messages with an ellipsis, so match on a
        # prefix that survives truncation.
        for override, message in (
            (("-DREPLAY_DIAL_PROBE_HOLD=1", "-DREPLAY_DIAL_PROBE_DURATION_MS=0"), "duration"),
            (("-DREPLAY_DIAL_PROBE_HOLD=1", "-DREPLAY_DIAL_PROBE_DURATION_MS=5001"), "duration"),
            (("-DREPLAY_DIAL_PROBE_HOLD=1", "-DREPLAY_DIAL_PROBE_VALUE=33"), "Probe value exceeds"),
            (("-DREPLAY_DIAL_PROBE_HOLD=1", "-DREPLAY_DIAL_PROBE_VALUE=-33"), "Probe value exceeds"),
            # A hold that cannot return to zero before the window ends.
            (("-DREPLAY_SECONDS=7",), "before window end"),
            # Holding is an explicit opt-in: without it the pulse bound still applies,
            # so the 3000 ms duration of this candidate must be rejected.
            (("-UREPLAY_DIAL_PROBE_HOLD", "-DREPLAY_DIAL_PROBE_HOLD=0"), "duration"),
        ):
            with self.subTest(override=override):
                self.assert_rejected("replay_gate0_hold16_echo", override, message)
        # Holding must be opt-in, and the opt-in must not change the value or the rate:
        # the same duration is accepted as a hold and rejected as a pulse.
        start, duration, guards, selector = self.probe_timeline("replay_gate0_hold16_echo")
        self.assertIn("200ms as a pulse", guards)
        subprocess.run([str(COMPILER), "-std=c++11", "-fsyntax-only", "-x", "c++", "-"],
                       input=self.compiled_code(start, duration, guards, selector),
                       text=True, encoding="utf-8", capture_output=True, check=True)

    def test_pair_changes_only_the_delay(self):
        source_a, flags_a = settings("replay_gate100_echo")
        source_b, flags_b = settings("replay_gate0_echo")
        self.assertEqual(source_a, source_b)
        trim = lambda flags: [f for f in flags if not f.startswith("-DREPLAY_STEADY_DELAY_MS=")]
        self.assertEqual(trim(flags_a), trim(flags_b))
        for flag in ("-DTX_QUEUE_LEN=8", "-DREPLAY_ECHO_FRAMES=1",
                     "-DREPLAY_SKIP_BOOT_MESSAGES=1", "-DREPLAY_LOG_CONFIG=1",
                     "-DREPLAY_SINGLE_ROUTE=1", "-DREPLAY_SEQ_START=0",
                     "-DREPLAY_SEQ_STEP=0", "-DREPLAY_INTERVAL_MS=500"):
            self.assertIn(flag, flags_a)
        self.assertIn("-DREPLAY_STEADY_DELAY_MS=100", flags_a)
        self.assertIn("-DREPLAY_STEADY_DELAY_MS=0", flags_b)

    def test_stable_candidate_waits_per_fragment_and_protects_uninstall(self):
        _, flags = settings("replay_gate0_stable_echo")
        self.assertNotIn("-DTX_QUEUE_LEN=8", flags)
        self.assertIn("-DTX_QUEUE_LEN=0", flags)
        self.assertIn("-DREPLAY_LEGACY_SEND=0", flags)
        expression, output = selected_gate("replay_gate0_stable_echo")
        self.assertEqual(expression, "true")
        self.assertIn("general.tx_queue_len = replay ? 0 : 0;", output)
        sender = output[output.index("static bool send_packet("):output.index("static bool send_m0001_request(")]
        self.assertLess(sender.index("for (uint8_t index"), sender.index("bool idle = false"))
        self.assertLess(sender.index("if (!idle)"), sender.index("twai_transmit(&message, 0)"))
        receiver = output[output.index("static void receive_task("):output.index("static void write_led(")]
        self.assertLess(receiver.index("xSemaphoreTake"), receiver.index("twai_receive"))
        self.assertGreater(receiver.rindex("xSemaphoreGive"), receiver.index("twai_receive"))
        switch = output[output.index("static bool set_mode("):output.index("static uint32_t tx_backoff_ticks")]
        self.assertLess(switch.index("xSemaphoreTake"), switch.index("twai_driver_uninstall"))
        self.assertGreater(switch.index("xSemaphoreGive"), switch.index("twai_start"))
        self.assertIn("driver_switching.store(true)", switch)
        self.assertIn("driver_switching.store(false)", switch)
        _, legacy = selected_gate("replay_gate0_echo")
        legacy_sender = legacy[legacy.index("static bool send_packet("):legacy.index("static bool send_m0001_request(")]
        self.assertNotIn("bool idle = false", legacy_sender)
        self.assertNotIn("xSemaphoreTake", legacy)

    def test_first_opportunity_and_existing_defaults(self):
        cases = {"replay_gate100_echo": False, "replay_gate0_echo": True,
                 "replay_bisect5": False, "replay_s4_auto": False,
                 "replay_s1_auto": True, "replay_hist_echo": True}
        for env, first_allowed in cases.items():
            with self.subTest(env=env):
                expression, output = selected_gate(env)
                if env.startswith("replay_gate"):
                    self.assertIn("general.tx_queue_len = replay ? 8 : 0;", output)
                    self.assertIn("message.self = 1;", output)
                code = ("constexpr bool allowed(unsigned long since_restart) { return "
                        + expression + "; }\n"
                        + "static_assert(allowed(0) == " + str(first_allowed).lower()
                        + ", \"first request opportunity\");\n"
                        + "static_assert(allowed(100), \"100 ms boundary\");\n")
                if not first_allowed:
                    code += "static_assert(!allowed(99), \"before boundary\");\n"
                subprocess.run([str(COMPILER), "-std=c++11", "-fsyntax-only", "-x", "c++", "-"],
                               input=code, text=True, encoding="utf-8",
                               capture_output=True, check=True)


if __name__ == "__main__":
    unittest.main()
