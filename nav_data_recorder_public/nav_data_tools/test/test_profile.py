"""Profile lookup: by name, by path, fallback to the default profile, $NAV_DATA_PROFILE."""
import json
import os

import pytest

from nav_data_tools import profile as P

SHIPPED = os.path.join(os.path.dirname(__file__), '..', '..', 'profiles')


@pytest.fixture
def root(tmp_path):
    for name, files in {'default': P.FILES, 'myrobot': ('topics.yaml',)}.items():
        (tmp_path / name).mkdir()
        for f in files:
            (tmp_path / name / f).write_text(f'{name}\n')
    return str(tmp_path)


def test_by_name_and_fallback(root, monkeypatch):
    monkeypatch.delenv('NAV_DATA_PROFILE', raising=False)
    assert P.profile_file('topics.yaml', 'myrobot', root) == os.path.join(root, 'myrobot', 'topics.yaml')
    # myrobot has no nightly_rules.yaml: the default profile's is used
    assert P.profile_file('nightly_rules.yaml', 'myrobot', root) == \
        os.path.join(root, 'default', 'nightly_rules.yaml')
    assert P.profile_dir(None, root) == os.path.join(root, 'default')
    assert P.available(root) == ['default', 'myrobot']


def test_env_and_path(root, monkeypatch, tmp_path_factory):
    monkeypatch.setenv('NAV_DATA_PROFILE', 'myrobot')
    assert P.profile_dir(None, root) == os.path.join(root, 'myrobot')
    outside = tmp_path_factory.mktemp('elsewhere')
    (outside / 'params.yaml').write_text('x')
    assert P.profile_file('params.yaml', str(outside), root) == str(outside / 'params.yaml')
    assert P.profile_file('topics.yaml', str(outside), root) == os.path.join(root, 'default', 'topics.yaml')


def test_unknown(root):
    with pytest.raises(P.ProfileError, match='installed: default, myrobot'):
        P.profile_dir('nope', root)
    with pytest.raises(P.ProfileError):
        P.profile_dir('./nope', root)


def test_shipped_profiles_complete():
    """The default profile has every file; every profile's layout is valid JSON."""
    for f in P.FILES:
        assert os.path.isfile(os.path.join(SHIPPED, 'default', f)), f
    for name in P.available(SHIPPED):
        lay = os.path.join(SHIPPED, name, 'nav_data_analysis.json')
        if os.path.isfile(lay):
            with open(lay) as fh:
                json.load(fh)
