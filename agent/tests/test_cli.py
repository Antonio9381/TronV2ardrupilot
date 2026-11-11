import json

from tron.cli import main

from .test_mavlink_loopback import FakeAutopilot, free_port


def run_cli(tmp_path, *args):
    port = free_port()
    cfg = tmp_path / "c.toml"
    cfg.write_text(f'[link]\nconn = "udpin:127.0.0.1:{port}"\nconnect_timeout_s = 5\n'
                   f'[log]\ndirectory = "{tmp_path / "logs"}"\n')
    ap = FakeAutopilot(port)
    try:
        return main(["-c", str(cfg), *args]), ap
    finally:
        ap.close()


def test_status(tmp_path, capsys):
    code, _ = run_cli(tmp_path, "status")
    assert code == 0
    out = json.loads(capsys.readouterr().out)
    assert out["sats"] == 17 and out["mode"] == "STABILIZE"
    assert list((tmp_path / "logs").iterdir())


def test_goto_refused_when_not_guided(tmp_path, capsys):
    code, ap = run_cli(tmp_path, "goto", "-33.8881", "151.1871", "10")
    assert code == 2
    assert "rejected" in capsys.readouterr().out
    assert ap.targets == []


def test_sitl_takeoff_refused_on_real_vehicle(tmp_path):
    port = free_port()
    cfg = tmp_path / "c.toml"
    cfg.write_text(f'[link]\nconn = "udpin:127.0.0.1:{port}"\n[log]\ndirectory = "{tmp_path}"\n')
    ap = FakeAutopilot(port)
    del ap.params["SIM_SPEEDUP"]
    try:
        assert main(["-c", str(cfg), "sitl-takeoff", "5"]) == 2
        assert not ap.armed
    finally:
        ap.close()


def test_check_objective(tmp_path, capsys):
    p = tmp_path / "o.json"
    p.write_text('{"lat": -33.88, "long": 151.18, "alt": 8, "colour": "red"}')
    assert main(["check-objective", str(p)]) == 0
    p.write_text('{"lat": -33.88, "alt": 8}')
    assert main(["check-objective", str(p)]) == 2


def test_mission_without_camera_ignores_colour(tmp_path, capsys):
    obj = tmp_path / "o.json"
    obj.write_text('{"lat": -33.888, "long": 151.187, "alt": 10, "colour": "red"}')
    code, ap = run_cli(tmp_path, "mission", str(obj))
    out = capsys.readouterr()
    assert "ignoring colour red" in out.err
    assert "phase=transit" in out.out
    assert ap.targets == []
