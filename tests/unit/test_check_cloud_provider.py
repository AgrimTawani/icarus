"""The provider check and the cloud replay: pure helpers and whole runs against fakes. No provider is contacted."""
import importlib.util
import json
from pathlib import Path

import pytest
from PIL import Image

SCRIPTS = Path(__file__).resolve().parents[2] / "scripts/autonomy"
KEY = "sk-secret-key"


def load(name):
    spec = importlib.util.spec_from_file_location(f"{name}_under_test", SCRIPTS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


check_cloud_provider = load("check_cloud_provider")
replay = load("replay_decision_latency")

TAKEOFF = {"role": "assistant", "content": None, "tool_calls": [
    {"id": "1", "type": "function", "function": {"name": "takeoff", "arguments": '{"target_altitude_agl_m": 3}'}}]}
STATS = {"tokens_prompt": 5000, "tokens_cached": 4000, "tokens_prefilled": 1000, "tokens_predicted": 20,
         "predicted_per_second": None, "retries": 0, "provider_host": "api.example.com", "model": "m"}


def test_check_summary_fails_when_any_check_fails():
    assert check_cloud_provider.summarise([("catalogue", True, 900), ("box", False, 1500)]) == (1, "FAIL box")


def test_check_summary_passes_only_when_all_pass():
    assert check_cloud_provider.summarise([("a", True, 1), ("b", True, 2)]) == (0, "PASS")


def test_check_summary_never_counts_a_skip_as_a_pass():
    status, verdict = check_cloud_provider.summarise([("a", True, 1), ("box", None, 0)])
    assert status == 0 and verdict.startswith("NO FAILURES") and "box" in verdict and verdict != "PASS"
    status, verdict = check_cloud_provider.summarise([("a", False, 1), ("box", None, 0)])
    assert status == 1 and verdict == "FAIL a; skipped (not verified): box"


def test_result_lines_carry_status_latency_and_tokens():
    line = check_cloud_provider.format_line("catalogue_takeoff", True, 912, "tokens prompt=5 cached=0 out=2", "takeoff {}")
    assert line == "PASS catalogue_takeoff  912 ms  tokens prompt=5 cached=0 out=2  takeoff {}"
    assert check_cloud_provider.format_line("pixel_box", False, 40).startswith("FAIL pixel_box  40 ms")
    assert (check_cloud_provider.format_line("pixel_box", None, 0, "", "frames not found: /x/f02.jpg")
            == "SKIP pixel_box (frames not found: /x/f02.jpg)")


def test_token_counts_are_n_a_when_the_runtime_reports_none():
    assert check_cloud_provider.format_tokens(None) == "tokens n/a"
    assert check_cloud_provider.format_tokens(STATS) == "tokens prompt=5000 cached=4000 out=20"


@pytest.mark.parametrize("box, ok", [
    ([10, 20, 300, 400], True), ([0, 0, 640, 480], True), ([0, 0, 641, 480], False), ([-1, 0, 5, 5], False),
    ([5, 5, 5, 9], False), ([9, 5, 5, 9], False), ([1, 2, 3], False), (None, False), ([1, 2, "3", 4], False)])
def test_box_must_be_inside_the_image(box, ok):
    assert check_cloud_provider.box_inside(box, 640, 480) is ok


def test_first_call_reads_name_and_arguments():
    assert check_cloud_provider.first_call(TAKEOFF) == ("takeoff", {"target_altitude_agl_m": 3})
    with pytest.raises(ValueError, match="no tool call.*'I will take off'"):
        check_cloud_provider.first_call({"role": "assistant", "content": "I will take off"})
    bad = {"tool_calls": [{"function": {"name": "takeoff", "arguments": "{oops"}}]}
    with pytest.raises(ValueError, match="not JSON"):
        check_cloud_provider.first_call(bad)


def test_frames_are_found_as_jpg_or_png_else_none(tmp_path):
    (tmp_path / "f02.jpg").write_bytes(b"x")
    (tmp_path / "f13.png").write_bytes(b"x")
    assert check_cloud_provider.find_frame(tmp_path, "f02") == tmp_path / "f02.jpg"
    assert check_cloud_provider.find_frame(tmp_path, "f13") == tmp_path / "f13.png"
    assert check_cloud_provider.find_frame(tmp_path, "f00") is None


def test_the_real_catalogue_and_request_validate_a_takeoff():
    messages, tools = check_cloud_provider.takeoff_request()
    names = [tool["function"]["name"] for tool in tools]
    assert "takeoff" in names and messages[0]["role"] == "system" and "Take off" in messages[1]["content"]
    from python.dcm.agent_tools import validate_call
    validate_call("takeoff", {"target_altitude_agl_m": 3}, tools=tools)


class FakeVision:
    """Answers condition questions from `found` in order, and box questions with a box."""

    def __init__(self, found):
        self.artifact = {"family": "cloud", "grounding_format": "json_norm1000_xyxy", "resident": True}
        self.found = iter(found)

    def complete(self, image_b64, question, prefill, n_predict, *, grammar=None, timeout_s=45.0):
        if "Find the" in question:
            return '{"bbox": [100, 200, 500, 1000]}'
        return json.dumps({"seen": "x", "found": next(self.found)})


class FakeCloud:
    """Stands in for CloudRuntime: no network."""

    def __init__(self, reply=TAKEOFF, fail_start=None, found=(True, False)):
        self.reply, self.fail_start, self.vision_server = reply, fail_start, FakeVision(found)
        self.last_stats, self.started, self.stopped = None, False, False

    def start(self):
        if self.fail_start:
            raise RuntimeError(self.fail_start)
        self.started = True

    def decide(self, messages, tools, *, concise=False):
        self.last_stats = STATS
        return self.reply

    def vision(self):
        return self.vision_server

    def stop(self):
        self.stopped = True


RECORD = {"model": "gemma4", "provider_host": "api.example.com", "vision_model": "gemma4",
          "vision_provider_host": "api.example.com", "vision": {"grounding_format": "json_norm1000_xyxy"}}


def frames(tmp_path, names=("f00", "f02")):
    for name in names:
        Image.new("RGB", (1000, 500)).save(tmp_path / f"{name}.png")
    labels = tmp_path / "labels.json"
    labels.write_text(json.dumps({"f00": {"building": False}, "f02": {"building": True}}))
    return labels


def run_main(monkeypatch, capsys, runtime, tmp_path, labels, extra=()):
    monkeypatch.setattr(check_cloud_provider, "make_runtime", lambda kind, manifest, **kw: (runtime, RECORD))
    code = check_cloud_provider.main(["--frames-dir", str(tmp_path), "--labels", str(labels), *extra])
    return code, capsys.readouterr()


def test_all_checks_pass_against_a_well_behaved_provider(monkeypatch, capsys, tmp_path):
    labels = frames(tmp_path)
    runtime = FakeCloud()
    code, out = run_main(monkeypatch, capsys, runtime, tmp_path, labels)
    lines = out.out.splitlines()
    assert code == 0 and lines[-1] == "RESULT PASS"
    assert lines[0] == ("Chat model gemma4 via api.example.com; camera model gemma4 via api.example.com "
                        "(grounding json_norm1000_xyxy)")
    statuses = [line.split()[:2] for line in lines[1:-1]]
    assert statuses == [["PASS", name] for name in check_cloud_provider.CHECK_NAMES]
    assert "tokens prompt=5000 cached=4000 out=20" in lines[2] and "takeoff" in lines[2]
    assert runtime.stopped and KEY not in out.out + out.err


def test_a_wrong_answer_fails_the_run(monkeypatch, capsys, tmp_path):
    labels = frames(tmp_path)
    runtime = FakeCloud(found=(True, True))  # also says "building" on the road frame
    code, out = run_main(monkeypatch, capsys, runtime, tmp_path, labels)
    assert code == 1 and out.out.splitlines()[-1] == "RESULT FAIL condition_false"
    assert "FAIL condition_false" in out.out and "found=True expected=False" in out.out


def test_missing_frames_are_skipped_not_passed(monkeypatch, capsys, tmp_path):
    labels = frames(tmp_path, names=())
    code, out = run_main(monkeypatch, capsys, FakeCloud(), tmp_path, labels)
    lines = out.out.splitlines()
    assert code == 0 and "SKIP condition_true (frames not found:" in out.out
    assert "SKIP pixel_box (frames not found:" in out.out
    assert lines[-1].startswith("RESULT NO FAILURES, but skipped (not verified): condition_true")


def test_a_reply_without_a_tool_call_fails_the_catalogue_check_and_skips_validation(monkeypatch, capsys, tmp_path):
    labels = frames(tmp_path)
    code, out = run_main(monkeypatch, capsys, FakeCloud(reply={"role": "assistant", "content": "ok"}), tmp_path, labels)
    assert code == 1 and "FAIL catalogue_takeoff" in out.out and "no tool call in the reply" in out.out
    assert "SKIP validate_call (no call to validate" in out.out


def test_a_wrong_tool_fails_the_catalogue_check(monkeypatch, capsys, tmp_path):
    labels = frames(tmp_path)
    wrong = {"role": "assistant", "tool_calls": [{"function": {"name": "hold", "arguments": "{}"}}]}
    code, out = run_main(monkeypatch, capsys, FakeCloud(reply=wrong), tmp_path, labels)
    assert code == 1 and "FAIL catalogue_takeoff" in out.out


def test_an_unreachable_provider_stops_after_the_first_failure(monkeypatch, capsys, tmp_path):
    labels = frames(tmp_path)
    runtime = FakeCloud(fail_start="cloud provider api.example.com rejected the API key (HTTP 401)")
    code, out = run_main(monkeypatch, capsys, runtime, tmp_path, labels)
    lines = out.out.splitlines()
    assert code == 1 and len(lines) == 3 and "FAIL provider_reachable" in lines[1] and "HTTP 401" in lines[1]
    assert runtime.stopped


def test_missing_settings_fail_cleanly_naming_the_variable(monkeypatch, capsys):
    for name in ("ICARUS_LLM_BASE_URL", "ICARUS_LLM_API_KEY", "ICARUS_LLM_MODEL"):
        monkeypatch.delenv(name, raising=False)
    from python.model_runtime.cloud_config import CloudSettings

    monkeypatch.setattr(check_cloud_provider, "make_runtime",
                        lambda kind, manifest, **kw: CloudSettings.from_manifest(manifest, environ={}))
    assert check_cloud_provider.main([]) == 2
    err = capsys.readouterr().err
    assert "ICARUS_LLM_BASE_URL is not set" in err and "Traceback" not in err


# ---- replay --runtime cloud ----

def write_episode(root, decisions):
    root.mkdir()
    with (root / "events.jsonl").open("w") as events:
        for payload in decisions:
            events.write(json.dumps({"kind": "agent_decision", "payload": payload}) + "\n")
    return root


def recorded(name, **arguments):
    return {"messages": [{"role": "system", "content": "s"}, {"role": "user", "content": "go"}],
            "tools": [{"type": "function", "function": {"name": name}}], "latency_ms": 700,
            "response": {"tool_calls": [{"function": {"name": name, "arguments": json.dumps(arguments)}}]}}


class FakeReplayRuntime(FakeCloud):
    def __init__(self, calls):
        super().__init__()
        self.calls, self.sent = list(calls), []

    def decide(self, messages, tools, *, concise=False):
        self.sent.append((messages, tools))
        name, arguments = self.calls.pop(0)
        self.last_stats = STATS
        return {"role": "assistant", "content": "", "tool_calls": [
            {"function": {"name": name, "arguments": json.dumps(arguments)}}]}


def replay_cloud(monkeypatch, capsys, tmp_path, runtime, *args):
    episode = write_episode(tmp_path / "ep", [recorded("takeoff", target_altitude_agl_m=3), recorded("hold")])
    monkeypatch.setattr(replay, "make_runtime", lambda kind, manifest, **kw: (runtime, RECORD))
    monkeypatch.setattr("sys.argv", ["replay", str(episode), "--runtime", "cloud", *args])
    replay.main()
    return [json.loads(line) for line in capsys.readouterr().out.splitlines() if line.startswith("{")]


def test_cloud_replay_reports_the_same_agreement_fields(monkeypatch, capsys, tmp_path):
    runtime = FakeReplayRuntime([("takeoff", {"target_altitude_agl_m": 3}), ("land", {})])
    rows = replay_cloud(monkeypatch, capsys, tmp_path, runtime)
    summary = rows[-1]
    assert summary["same_tool_as_recorded"] == 1 and summary["same_call_as_recorded"] == 1
    assert summary["non_finish_steps"] == 2 and summary["gen_tokens"] == 40 and summary["steps"] == 2
    assert summary["provider_host"] == "api.example.com" and summary["model"] == "gemma4"
    assert rows[0]["tool"] == "takeoff" and rows[0]["cached"] == 4000 and rows[0]["prefilled"] == 1000
    assert rows[0]["prompt_total"] == 5000 and rows[0]["recorded_ms"] == 700
    assert runtime.started and runtime.stopped and len(runtime.sent) == 2


def test_cloud_replay_uses_the_recorded_request_unchanged(monkeypatch, capsys, tmp_path):
    runtime = FakeReplayRuntime([("takeoff", {}), ("hold", {})])
    replay_cloud(monkeypatch, capsys, tmp_path, runtime)
    messages, tools = runtime.sent[0]
    assert messages == recorded("takeoff")["messages"] and tools == recorded("takeoff")["tools"]


def test_replay_defaults_to_the_local_runtime():
    assert replay.build_parser().parse_args(["episode"]).runtime == "local"
    assert replay.build_parser().parse_args(["episode", "--runtime", "llama"]).runtime == "local"
    assert replay.build_parser().parse_args(["episode", "--runtime", "cloud"]).runtime == "cloud"


def test_local_replay_still_posts_the_llama_body_and_reads_timings(monkeypatch, capsys, tmp_path):
    import io

    sent = []

    class Llama:
        endpoint = "http://127.0.0.1:1"

        def __init__(self, descriptor, extra_args=None):
            self.extra_args = extra_args

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

        def _mission_slot(self):
            return {"id_slot": 0}

    def urlopen(request, timeout):
        sent.append(json.loads(request.data))
        reply = {"choices": [{"message": {"tool_calls": [{"function": {"name": "hold", "arguments": "{}"}}]}}],
                 "usage": {"prompt_tokens": 90}, "timings": {"prompt_n": 10, "cache_n": 80, "prompt_ms": 5.4,
                                                              "predicted_n": 3, "predicted_ms": 7.2}}
        return io.BytesIO(json.dumps(reply).encode())

    episode = write_episode(tmp_path / "ep", [recorded("hold")])
    monkeypatch.setattr(replay, "LlamaCppRuntime", Llama)
    monkeypatch.setattr(replay.RuntimeDescriptor, "from_manifest", classmethod(lambda cls, *a, **k: object()))
    monkeypatch.setattr(replay.urllib.request, "urlopen", urlopen)
    monkeypatch.setattr("sys.argv", ["replay", str(episode)])
    replay.main()
    rows = [json.loads(line) for line in capsys.readouterr().out.splitlines() if line.startswith("{")]
    assert sent[0]["cache_prompt"] is True and sent[0]["parallel_tool_calls"] is True and sent[0]["id_slot"] == 0
    assert sent[0]["chat_template_kwargs"] == {"enable_thinking": False}
    assert (rows[0]["prompt_total"], rows[0]["prefilled"], rows[0]["cached"], rows[0]["prefill_ms"],
            rows[0]["gen_tokens"], rows[0]["gen_ms"]) == (90, 10, 80, 5, 3, 7)
    assert "provider_host" not in rows[-1] and rows[-1]["same_tool_as_recorded"] == 1
