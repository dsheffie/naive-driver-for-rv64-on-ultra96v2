#!/bin/sh
# newport_vnc.sh -- the X/VNC server the Newport window draws on (mips-axi
# --newport-display :2), separate from irix_vnc.sh's :1 (IRIX's X-over-network desktop).
#
#   mips-axi (Newport model) --Xlib, unix socket--> Xtightvnc :2 <--VNC 5902-- viewer
#
# X is local only (-nolisten tcp + a per-run MIT-MAGIC-COOKIE); VNC uses the same
# password file as :1.  Run as root on the board.
#
# Usage:  ./newport_vnc.sh start | stop | status
#   then: DISPLAY=:2 XAUTHORITY=/home/root/.newportvnc.auth mips-axi ... --newport-display :2
set -u
DISP=2
VNCPORT=$((5900 + DISP))
GEOM=1280x1024                      # the Newport display
DEPTH=24                            # newport_arm.cc needs a 24-bit RGB888 visual
AUTHF=/home/root/.newportvnc.auth
PIDF=/var/run/newportvnc.pid
LOGF=/var/log/newportvnc.log
PASSF=/home/root/.vnc/passwd

log() { echo "[newport_vnc] $*"; }

running() {
	[ -f "$PIDF" ] && kill -0 "$(cat $PIDF)" 2>/dev/null \
		&& [ "$(readlink /proc/$(cat $PIDF)/exe)" = "$(command -v Xtightvnc)" ]
}

start() {
	if running; then
		log "already running (pid $(cat $PIDF))"
		return 0
	fi
	[ -f "$PASSF" ] || { log "FATAL: no VNC password file $PASSF (run vncpasswd)"; return 1; }
	cookie=$(od -An -N16 -tx1 /dev/urandom | tr -d ' \n')
	rm -f "$AUTHF"; touch "$AUTHF"; chmod 600 "$AUTHF"
	xauth -f "$AUTHF" add ":$DISP" MIT-MAGIC-COOKIE-1 "$cookie"
	log "starting Xtightvnc :$DISP ($GEOM x$DEPTH), VNC on 0.0.0.0:$VNCPORT"
	Xtightvnc ":$DISP" -desktop "Newport on henry" -geometry "$GEOM" -depth "$DEPTH" \
		-nolisten tcp -auth "$AUTHF" -rfbauth "$PASSF" -rfbport "$VNCPORT" -alwaysshared \
		> "$LOGF" 2>&1 &
	echo $! > "$PIDF"
	i=0
	while [ $i -lt 20 ]; do
		DISPLAY=":$DISP" XAUTHORITY="$AUTHF" xdpyinfo > /dev/null 2>&1 && break
		sleep 0.5; i=$((i + 1))
	done
	if ! DISPLAY=":$DISP" XAUTHORITY="$AUTHF" xdpyinfo > /dev/null 2>&1; then
		log "FATAL: X server not answering"
		tail -5 "$LOGF"
		return 1
	fi
	log "up (pid $(cat $PIDF)); run mips-axi with DISPLAY=:$DISP XAUTHORITY=$AUTHF --newport-display :$DISP"
}

stop() {
	if running; then
		p=$(cat "$PIDF")
		kill "$p"
		log "stopped Xtightvnc pid $p"
	else
		log "not running"
	fi
	rm -f "$PIDF"
}

status() {
	if running; then
		log "running pid $(cat $PIDF); VNC fpga.local:$VNCPORT, display :$DISP"
	else
		log "not running"
	fi
}

case "${1:-}" in
	start) start ;;
	stop) stop ;;
	status) status ;;
	*) echo "usage: $0 start|stop|status"; exit 1 ;;
esac
