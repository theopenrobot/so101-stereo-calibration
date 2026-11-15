#!/bin/sh
# Start, stop or check the station on this machine. The same script on a Mac and on Linux (a Raspberry Pi 5).
#
#   ./run.sh check     what this machine resolves to: Python, camera address, arm ports, data folder. Starts nothing.
#   ./run.sh start     the station and its page (http://127.0.0.1:18771/)
#   ./run.sh camera    the camera program (Linux only; the camera must be plugged into this machine)
#   ./run.sh status
#   ./run.sh stop      stops what this script started
#
# Settings come from station.env (copy station.env.example). Nothing is installed and nothing starts at boot.
set -e
cd "$(dirname "$0")"
setting() { python3 station/hostconf.py | python3 -c "import sys, json; print(json.load(sys.stdin)['$1'])"; }
PY=$(setting python)          # STATION_PY in station.env: the Python with lerobot, opencv-python and numpy
DATA=$(setting data)
RUN="$DATA/run"
mkdir -p "$RUN"

running() { [ -f "$RUN/$1.pid" ] && kill -0 "$(cat "$RUN/$1.pid")" 2>/dev/null; }

launch() {     # launch <name> <command...>
    name=$1; shift
    if running "$name"; then echo "$name: already running (pid $(cat "$RUN/$name.pid"))"; return 0; fi
    nohup "$@" > "$RUN/$name.log" 2>&1 &
    echo $! > "$RUN/$name.pid"
    sleep 2
    if running "$name"; then echo "$name: started (pid $(cat "$RUN/$name.pid")), log $RUN/$name.log"
    else echo "$name: did not stay up. Last lines of $RUN/$name.log:"; tail -5 "$RUN/$name.log"; return 1; fi
}

halt() {
    if running "$1"; then kill "$(cat "$RUN/$1.pid")"; echo "$1: stopped"; else echo "$1: not running"; fi
    rm -f "$RUN/$1.pid"
}

case "${1:-}" in
    check)
        $PY station/hostconf.py
        $PY arm/so101_host.py
        ;;
    start)
        launch station $PY station/calib_station.py
        launch page $PY station/calib_dashboard.py
        echo "Open the page at the address in $RUN/page.log."
        ;;
    camera)
        if [ "$(uname -s)" != "Linux" ]; then
            echo "The camera program needs Linux (V4L2). Plug the camera into a Pi or a Linux PC and run ./run.sh camera there,"
            echo "then set STATION_CAMERA_URL=http://<that machine>:8090 in station.env on the station machine."
            exit 1
        fi
        launch camera python3 camera/stereo_stream.py
        ;;
    status)
        for name in station page camera; do
            if running "$name"; then echo "$name: running (pid $(cat "$RUN/$name.pid"))"; else echo "$name: not running"; fi
        done
        ;;
    stop)
        halt page; halt station; halt camera
        ;;
    *)
        sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'
        exit 2
        ;;
esac
