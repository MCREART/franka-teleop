#!/bin/zsh
set -eu
cd "${0:A:h}"

if command -v uv >/dev/null 2>&1; then
    uv_bin="$(command -v uv)"
elif [[ -x "$HOME/.local/bin/uv" ]]; then
    uv_bin="$HOME/.local/bin/uv"
else
    echo "请先安装 uv，参见 README.md。"
    exit 1
fi

exec "$uv_bin" run --frozen mjpython -m server "$@"
