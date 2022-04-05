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
# tm1637d_XXX_scl (int):	Set SCL pin number;
# tm1637d_XXX_sda (int):	Set SDA pin number;
# tm1637d_XXX_digits (int):	Set disgits number.
# tm1637d_flags (str):		Set to "" by default.
#				Extra flags passed to start command.
#
# For a profile based configuration use variables like this:
#
# tm1637d_profiles="XXX"
# tm1637d_flags="-b"
# tm1637d_XXX_scl="0"
# tm1637d_XXX_sda="1"
# tm1637d_XXX_digits="6"

. /etc/rc.subr

name=tm1637d
rcvar=tm1637d_enable

load_rc_config $name

: ${tm1637d_enable:="NO"}
: ${tm1637d_scl:="0"}
: ${tm1637d_sda:="1"}
: ${tm1637d_digits:="4"}
: ${tm1637d_type:=""}
: ${tm1637d_flags:="-b"}

tm1637d_bin="/usr/local/sbin/tm1637d"

tm1637d_device_args=""
if [ -n "${tm1637d_profiles}" ]; then
	for profile in ${tm1637d_profiles}; do
		echo "===> tm1637d profile: ${profile}"
		eval _scl=\${tm1637d_${profile}_scl:-"${tm1637d_scl}"}
		eval _sda=\${tm1637d_${profile}_sda:-"${tm1637d_sda}"}
		eval _digits=\${tm1637d_${profile}_digits:-"${tm1637d_digits}"}
		eval _type=\${tm1637d_${profile}_type:-"${tm1637d_type}"}
		_device="scl=${_scl},sda=${_sda},digits=${_digits}"
		if [ "${_type}" == "clock" ]; then
			_device="${_device},clock"
		fi
		echo ${_device}
		tm1637d_device_args="${tm1637d_device_args} -d ${_device}"
	done
else
	tm1637d_device_args="-d scl=${tm1637d_scl},sda=${tm1637d_sda},digits=${tm1637d_digits}"
	if [ "${tm1637d_type}" == "clock" ]; then
		tm1637d_device_args="${tm1637d_device_args},clock"
	fi
fi

command=${tm1637d_bin}
command_args="${tm1637d_device_args} ${tm1637d_flags}"

run_rc_command "$@"
