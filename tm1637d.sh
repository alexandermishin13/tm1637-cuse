#!/bin/sh
#
# $FreeBSD: 2021-06-27 19:08:00Z mishin $
#
# PROVIDE: tm1637d
# REQUIRE: DAEMON
# KEYWORD: nojail shutdown
#
# Add the following lines to /etc/rc.conf to enable tm1637d:
#
# tm1637d_enable (bool):	Set to "NO"  by default.
#				Set to "YES" to enable tm1637d.
# tm1637d_profiles (str):	Set to "" by default.
#				Define your profiles here.
# tm1637d_XXX_scl (int):	Set SCL pin number.
# tm1637d_XXX_sda (int):	Set SDA pin number.
# tm1637d_flags (str):		Set to "" by default.
#				Extra flags passed to start command.
#
# For a profile based configuration use variables like this:
#
# tm1637d_profiles="XXX"
# tm1637d_flags="-b"
# tm1637d_XXX_scl="0"
# tm1637d_XXX_sda="1"

. /etc/rc.subr

name=tm1637d
rcvar=tm1637d_enable

load_rc_config $name

: ${tm1637d_enable:="NO"}
: ${tm1637d_flags:="-b"}

tm1637d_bin="/usr/local/sbin/tm1637d"

tm1637d_device_args=""
if [ -n "${tm1637d_profiles}" ]; then
	for profile in ${tm1637d_profiles}; do
		echo "===> tm1637d profile: ${profile}"
		eval tm1637d_scl=\${tm1637d_${profile}_scl:-"${tm1637d_scl}"}
		eval tm1637d_sda=\${tm1637d_${profile}_sda:-"${tm1637d_sda}"}
		echo "scl=${tm1637d_scl},sda=${tm1637d_sda}"
		tm1637d_device_args="${tm1637d_device_args} -d scl=${tm1637d_scl},sda=${tm1637d_sda}"
	done
else
	tm1637d_device_args="-d scl=${tm1637d_scl},sda=${tm1637d_sda}"
fi

command=${tm1637d_bin}
command_args="${tm1637d_device_args} ${tm1637d_flags}"

run_rc_command "$@"
