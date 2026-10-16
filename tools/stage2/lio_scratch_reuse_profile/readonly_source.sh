#!/bin/bash
set -e
profile_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
profile_source="$profile_dir/source"
mount --bind "$profile_source" "$profile_source"
mount -o remount,bind,ro "$profile_source"
exec "$@"
