#!/bin/bash -efu
# Copyright (c) 2022 Vladimir D. Seleznev
# SPDX-License-Identifier: GPL-2.0
#
# AltHa test for nosuid feature

sysctl -q kernel.altha.nosuid.enabled >/dev/null || {
	echo >&2 "AltHa is not enabled, quitting"
	exit 2
}

ret=0

num_failed=0
num_tests=0

nosuid_enabled=kernel.altha.nosuid.enabled
nosuid_exeptions=kernel.altha.nosuid.exceptions

tmpdir="$(mktemp -d)"
cleanup()
{
	if [ -f "$tmpdir/tmp_mount_options" ] &&
	   [ -f "$tmpdir/tmp_mount_target" ]; then
		   mount -o remount,"$(cat "$tmpdir/tmp_mount_options")" \
			   "$(cat "$tmpdir/tmp_mount_target")"
	fi

	[ ! -f "$tmpdir/nosuid_enabled" ] ||
		sysctl "$nosuid_enabled=$(cat "$tmpdir/nosuid_enabled")"

	[ ! -f "$tmpdir/nosuid_exceptions" ] ||
		sysctl "$nosuid_exeptions=$(cat "$tmpdir/nosuid_exceptions")"

	rm -r "$tmpdir"
	exit "$@"
}
trap 'cleanup $?' EXIT QUIT INT ERR

save_altha_state()
{
	findmnt /tmp |sed 1d |while read -r target source fstype options; do
		echo "$options" > "$tmpdir/tmp_mount_options"
		echo "$target" > "$tmpdir/tmp_mount_target"
		mount -o remount,suid "$target"
	done

	sysctl "$nosuid_enabled" |cut -f3 -d' ' > "$tmpdir/nosuid_enabled"
	sysctl "$nosuid_exeptions" |cut -f3 -d' ' > "$tmpdir/nosuid_exceptions"
}

run_test()
{
	local test_cmd="$1"; shift
	local test_cond="$1"; shift

	while IFS=$'\t' read -r precond expres; do
		num_tests=$((num_tests + 1))

		eval "$precond"
		eval "$test_cmd" >"$tmpdir/result" 2>&1 ||:

		if [ "$(cat "$tmpdir/result")" != "$expres" ]; then
			echo >&2 "$test_cmd FAILED with $precond"
			echo >&2 "expected result: $expres"
			echo >&2 "actual result: $(cat "$tmpdir/result")"
			num_failed=$((num_failed + 1))
		fi
	done <"$test_cond"
}

check_setuid()
{
	install -pm4755 -t "$tmpdir" /usr/bin/id

	local nobody_uid
	nobody_uid="$(grep -E '^\<nobody\>' /etc/passwd |cut -f3 -d:)"

	cat <<EOF >"$tmpdir/setuid_test"
sysctl $nosuid_enabled=0	0
sysctl $nosuid_enabled=1	$nobody_uid
sysctl $nosuid_exeptions=$tmpdir/id	0
EOF


	run_test "setpriv --reuid nobody -- $tmpdir/id -u" "$tmpdir/setuid_test"
}

check_setcap()
{
	install -p -t "$tmpdir" /usr/bin/nc
	setcap cap_net_bind_service,cap_net_admin+ep "$tmpdir/nc"

	cat <<EOF >"$tmpdir/setcap_test"
sysctl $nosuid_enabled=0
sysctl $nosuid_enabled=1	nc: Permission denied
sysctl $nosuid_exeptions=$tmpdir/nc
EOF

	run_test "timeout 1 setpriv --reuid nobody -- $tmpdir/nc -l 9" "$tmpdir/setcap_test"
}

save_altha_state
check_setuid
check_setcap

if [ "$num_failed" -ne 0 ]; then
	echo >&2 "$num_failed of $num_tests tests FAILED"
	ret=1
else
	echo >&2 "All $num_tests tests succeed"
fi

exit $ret
