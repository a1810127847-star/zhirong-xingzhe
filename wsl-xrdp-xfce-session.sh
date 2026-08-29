#!/bin/sh

unset WAYLAND_DISPLAY
unset DBUS_SESSION_BUS_ADDRESS
unset SESSION_MANAGER

export XDG_SESSION_TYPE=x11
export GDK_BACKEND=x11
export QT_QPA_PLATFORM=xcb

exec dbus-launch --exit-with-session xfce4-session
