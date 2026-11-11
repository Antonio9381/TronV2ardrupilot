import json

import pytest

from tron.config import load_config
from tron.objective import ObjectiveError, load_objective, parse_objective


def test_defaults_are_safe(cfg):
    assert cfg.link.send_heartbeat is False
    assert cfg.limits.require_path_planning is True
    assert cfg.link.conn.endswith("14553")


def test_example_config_loads():
    from pathlib import Path
    cfg = load_config(Path(__file__).parent.parent / "config.example.toml")
    assert cfg.mission.end_action in ("hold", "rtl", "land")


def test_config_rejects_unknown(tmp_path):
    p = tmp_path / "c.toml"
    p.write_text("[limits]\nfence_radius = 5\n")
    with pytest.raises(ValueError, match="unknown setting"):
        load_config(p)
    p.write_text("[limits]\nalt_max_m = 'high'\n")
    with pytest.raises(ValueError, match="must be float"):
        load_config(p)
    p.write_text("[mission]\nend_action = 'flip'\n")
    with pytest.raises(ValueError):
        load_config(p)


def test_objective_from_sentence_dict():
    o = parse_objective({"lat": -33.8881, "long": 151.1871, "alt": 10, "colour": " Red ", "target": "tarp"})
    assert (o.lat, o.lon, o.alt, o.colour, o.target) == (-33.8881, 151.1871, 10.0, "red", "tarp")


def test_objective_without_colour():
    o = parse_objective({"latitude": 1, "lng": 2, "altitude": "8"})
    assert o.colour is None and o.alt == 8.0


@pytest.mark.parametrize("data,msg", [
    ({"long": 1, "alt": 5}, "missing lat"),
    ({"lat": 1, "long": 2, "lon": 2, "alt": 5}, "more than once"),
    ({"lat": 91, "long": 2, "alt": 5}, "lat out of range"),
    ({"lat": 1, "long": 2, "alt": float("nan")}, "finite"),
    ({"lat": 1, "long": 2, "alt": True}, "number"),
    ({"lat": 1, "long": 2, "alt": 5, "colour": "chartreuse"}, "colour must be"),
    ({"lat": 1, "long": 2, "alt": 5, "speed": 99}, "unknown keys"),
])
def test_objective_rejects(data, msg):
    with pytest.raises(ObjectiveError, match=msg):
        parse_objective(data)


def test_load_objective_file(tmp_path):
    p = tmp_path / "o.json"
    p.write_text(json.dumps({"lat": 1, "long": 2, "alt": 5}))
    assert load_objective(p).lon == 2
    p.write_text("{nope")
    with pytest.raises(ObjectiveError, match="invalid JSON"):
        load_objective(p)
