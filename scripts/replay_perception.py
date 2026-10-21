#!/usr/bin/env python3
"""Replay only camera input on an explicitly selected offline ROS domain."""

import argparse
import os
from pathlib import Path


def command(bag):
    return [
        'ros2',
        'bag',
        'play',
        str(bag),
        '--clock',
        '--delay',
        '3',
        '--disable-keyboard-controls',
        '--topics',
        '/camera/image_raw',
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path, help='session directory or bag directory')
    parser.add_argument(
        '--domain-id',
        type=int,
        required=True,
        help='offline domain, different from current live domain',
    )
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 101 or args.domain_id == int(
        os.environ.get('ROS_DOMAIN_ID', '0')
    ):
        parser.error('choose an offline domain in 0..101 different from the current ROS_DOMAIN_ID')
    bag = args.session / 'bag' if (args.session / 'bag').is_dir() else args.session
    if not (bag / 'metadata.yaml').is_file():
        parser.error('bag metadata.yaml is missing')
    environment = dict(os.environ, ROS_DOMAIN_ID=str(args.domain_id))
    # exec preserves Ctrl-C/exit status. No arbitrary rosbag options can widen this allowlist.
    os.execvpe('ros2', command(bag), environment)


if __name__ == '__main__':
    main()
