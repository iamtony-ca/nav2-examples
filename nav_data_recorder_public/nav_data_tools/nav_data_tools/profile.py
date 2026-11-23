"""Find the files of a robot profile.

A profile is a directory with any of topics.yaml, params.yaml, nightly_rules.yaml and
nav_data_analysis.json. It is given by name (installed by the nav_data_profiles package as
share/nav_data_profiles/profiles/<name>/) or by path (anything containing '/'). Default: $NAV_DATA_PROFILE,
else 'default' (plain Nav2). A file the profile does not have is taken from the default profile.
"""
import os

DEFAULT = 'default'
FILES = ('topics.yaml', 'params.yaml', 'nightly_rules.yaml', 'nav_data_analysis.json')


class ProfileError(RuntimeError):
    pass


def profiles_root():
    """share/nav_data_profiles/profiles, or the source tree's profiles/ when not installed."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('nav_data_profiles'), 'profiles')
    except Exception:  # noqa: BLE001  (not installed / no ament index: running from source)
        return os.path.join(os.path.dirname(__file__), '..', '..', 'profiles')


def available(root=None):
    root = root or profiles_root()
    try:
        return sorted(d for d in os.listdir(root) if os.path.isdir(os.path.join(root, d)))
    except OSError:
        return []


def profile_name(profile=None):
    return profile or os.environ.get('NAV_DATA_PROFILE', '') or DEFAULT


def profile_dir(profile=None, root=None):
    p = os.path.expanduser(profile_name(profile))
    if '/' in p:
        if not os.path.isdir(p):
            raise ProfileError(f'profile directory not found: {p}')
        return os.path.abspath(p)
    root = root or profiles_root()
    d = os.path.join(root, p)
    if not os.path.isdir(d):
        raise ProfileError(f"profile '{p}' not found in {root} "
                           f"(installed: {', '.join(available(root)) or 'none'}; "
                           'rebuild nav_data_profiles after adding one, or pass a path)')
    return d


def profile_file(name, profile=None, root=None):
    """<profile>/<name>, else the default profile's <name>."""
    f = os.path.join(profile_dir(profile, root), name)
    if os.path.isfile(f):
        return f
    f = os.path.join(root or profiles_root(), DEFAULT, name)
    if os.path.isfile(f):
        return f
    raise ProfileError(f'{name} not found in profile {profile_name(profile)!r} nor in {DEFAULT!r}')


def main(argv=None):
    """nav_data_profile [profile] [file]: print the profile directory or the path of one file."""
    import argparse
    p = argparse.ArgumentParser(description=main.__doc__)
    p.add_argument('profile', nargs='?', default='', help='name or path (default: $NAV_DATA_PROFILE, else default)')
    p.add_argument('file', nargs='?', choices=FILES)
    p.add_argument('--list', action='store_true', help='list installed profiles')
    a = p.parse_args(argv)
    if a.list:
        print('\n'.join(available()))
        return 0
    try:
        print(profile_file(a.file, a.profile) if a.file else profile_dir(a.profile))
    except ProfileError as e:
        print(f'error: {e}')
        return 1
    return 0
