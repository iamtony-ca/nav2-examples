"""Every module imports and every console script has a main() (catches syntax errors in tools
that the other tests do not run, such as nav_data_session)."""
import importlib

import pytest


@pytest.mark.parametrize('name', ['common', 'codecs', 'export', 'maintenance', 'nightly', 'profile', 'prune', 'session'])
def test_module_imports(name):
    mod = importlib.import_module(f'nav_data_tools.{name}')
    if name in ('export', 'maintenance', 'nightly', 'profile', 'prune', 'session'):
        assert callable(mod.main)


@pytest.mark.parametrize('name', ['export', 'maintenance', 'nightly', 'profile', 'prune', 'session'])
def test_help_runs(name, capsys):
    mod = importlib.import_module(f'nav_data_tools.{name}')
    with pytest.raises(SystemExit) as e:
        mod.main(['--help'])
    assert e.value.code == 0
    assert 'usage' in capsys.readouterr().out
