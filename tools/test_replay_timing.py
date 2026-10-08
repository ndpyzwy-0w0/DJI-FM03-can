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


class ReplayTimingTests(unittest.TestCase):
    def test_motion_probe_boundaries_and_manual_start_guard(self):
        for env, value in (("replay_gate0_probe16_echo", 16),
                           ("replay_gate0_stable_50ms_echo", 0)):
            with self.subTest(env=env):
                _, output = selected_gate(env)
                selector = re.search(r"static constexpr int16_t replay_dial_value\([^}]+\}", output)[0]
                guards = "\n".join(re.findall(r"^static_assert\([^\n]+", output, re.M))
                code = "#include <stdint.h>\n" + guards + "\n" + selector
                for elapsed, expected in ((0, 0), (4999, 0), (5000, value),
                                          (5199, value), (5200, 0), (15000, 0),
                                          (4294967295, 0)):
                    code += (f'\nstatic_assert(replay_dial_value({elapsed}UL) == {expected}, '
                             '"probe boundary");')
                subprocess.run([str(COMPILER), "-std=c++11", "-fsyntax-only", "-x", "c++", "-"],
                               input=code, text=True, encoding="utf-8",
                               capture_output=True, check=True)
        _, output = selected_gate("replay_gate0_probe16_echo",
                                  ("-UREPLAY_AUTO_START", "-DREPLAY_AUTO_START=1"))
        guards = "\n".join(re.findall(r"^static_assert\([^\n]+", output, re.M))
        result = subprocess.run([str(COMPILER), "-std=c++11", "-fsyntax-only", "-x", "c++", "-"],
                                input=guards, text=True, encoding="utf-8", capture_output=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Motion probe requires manual start", result.stderr)

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
