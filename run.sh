#!/bin/sh
# 셈틀 실행기. 고친 게 있으면 다시 빌드하고 띄운다.
cd "$(dirname "$0")" || exit 1

if ! make -s 2>/tmp/semtle-build.log; then
    if command -v notify-send >/dev/null; then
        notify-send -u critical "셈틀: 빌드 실패" "$(tail -3 /tmp/semtle-build.log)"
    fi
    cat /tmp/semtle-build.log >&2
    [ -x ./semtle ] || exit 1        # 예전 실행 파일도 없으면 포기
fi

exec ./semtle "$@"
