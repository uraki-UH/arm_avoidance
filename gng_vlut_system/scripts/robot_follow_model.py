"""s・r・fの接続構成検査。単一入力元と循環経路の拒否。"""
import math
import re


def validate_profile(profile):
    if not isinstance(profile, dict) or set(profile) != {'label', 'simulator_source', 'simulator_mode', 'follower_source', 'follow_mode'}:
        raise ValueError('追従構成の項目不正')
    if (not isinstance(profile['label'], str) or
            profile['simulator_source'] not in ('none', 'r', 'f') or
            profile['simulator_mode'] not in ('manual', 'display', 'dynamics') or
            profile['follower_source'] not in ('none', 'r', 's') or
            profile['follow_mode'] not in ('absolute', 'relative')):
        raise ValueError('追従構成の入力元・方式不正')
    if (profile['simulator_source'] == 'none') != (profile['simulator_mode'] == 'manual'):
        raise ValueError('手動操作と外部入力の競合')
    if profile['simulator_source'] == 'f' and profile['follower_source'] == 's':
        raise ValueError('f → s → fの循環経路')
    return dict(profile)


def validate_config(config):
    if not isinstance(config, dict) or set(config) != {'roles', 'max_state_age_sec', 'profiles'}:
        raise ValueError('追従設定の項目不正')
    roles = config['roles']
    if (not isinstance(roles, dict) or set(roles) != {'s', 'r', 'f'} or
            any(not isinstance(topic, str) or re.fullmatch(r'(?:/[A-Za-z_][A-Za-z_0-9]*)+', topic) is None for topic in roles.values()) or
            len(set(roles.values())) != 3):
        raise ValueError('s・r・fのトピック重複または不足')
    if roles['s'] != '/robot_follow/simulator_state' or any(topic.startswith('/robot_follow/') for role, topic in roles.items() if role != 's'):
        raise ValueError('管理内部のトピックと実測入力の競合')
    age = config['max_state_age_sec']
    if type(age) not in (int, float) or not math.isfinite(age) or not 0 < age <= 5:
        raise ValueError('実測失効時間の設定不正')
    if not isinstance(config['profiles'], dict) or not config['profiles']:
        raise ValueError('追従構成の不足')
    for name, profile in config['profiles'].items():
        if not isinstance(name, str) or not name:
            raise ValueError('追従構成名の不正')
        validate_profile(profile)
    return config
