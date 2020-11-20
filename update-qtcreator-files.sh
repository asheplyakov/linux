#!/bin/sh
set -e
MYDIR="${0%/*}"
cd "$MYDIR"
fileslist=BaikalM-Linux-5.11.files

list_sources () {
	local arches="$*"
	for arch in $arches; do
		git ls-files "arch/$arch/**.[chS]" | grep -v -E 'boot/dts/'
	done
	git ls-files '*.[chS]' | grep -v -E '^(arch)|(Documentation)|(tools)|(virt)'
}

list_sources arm arm64 | sort -u > "${fileslist}"
