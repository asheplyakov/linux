%def_disable check

Name: kernel-image-mitx-xpa
Release: alt4


%define kernel_base_version	4.4
%define kernel_sublevel .215
%define kernel_extra_version	%nil
Version: %kernel_base_version%kernel_sublevel%kernel_extra_version

%define krelease	%release

%define flavour		%( s='%name'; printf %%s "${s#kernel-image-}" )
%define base_flavour	%( s='%flavour'; printf %%s "${s%%%%-*}" )
%define sub_flavour	%( s='%flavour'; printf %%s "${s#*-}" )

%define nprocs 4
# Build options
# You can change compiler version by editing this line:
%define kgcc_version	8

# Enable/disable SGML docs formatting
#%%if "%sub_flavour" == "def"
#%%def_enable docs
#%%else
#%%def_disable docs
#%%endif

#Remove oss
%def_disable oss
## Don't edit below this line ##################################

%define kversion	%kernel_base_version%kernel_sublevel%kernel_extra_version
%define modules_dir	/lib/modules/%kversion-%flavour-%krelease

%define kheaders_dir	%_prefix/include/linux-%kversion-%flavour
%define kbuild_dir	%_prefix/src/linux-%kversion-%flavour-%krelease
%define old_kbuild_dir	%_prefix/src/linux-%kversion-%flavour

%brp_strip_none /boot/*

Summary: The Linux kernel (the core of the Linux operating system)
License: GPL
Group: System/Kernel and hardware
Url: https://www.t-platforms.ru/upload/iblock/818/linux_source_4.4.174.tar.gz 
Packager: Kernel Maintainers Team <kernel@packages.altlinux.org>

Source: %name-%version.tar

ExclusiveArch: mipsel
ExclusiveOS: Linux

BuildRequires(pre): rpm-build-kernel
BuildRequires: flex
BuildRequires: libdb4-devel
BuildRequires: gcc%kgcc_version gcc%kgcc_version-c++
BuildRequires: gcc%kgcc_version-plugin-devel libgmp-devel libmpc-devel
BuildRequires: module-init-tools >= 3.16
BuildRequires: lzma-utils
BuildRequires: bc
BuildRequires: openssl-devel

%{?!_without_check:%{?!_disable_check:BuildRequires: qemu-system-x86 glibc-devel-static}}

%if_enabled docs
BuildRequires: xmlto transfig ghostscript
%endif

%if_enabled ccache
BuildRequires: ccache
%endif

%ifdef use_ccache
BuildRequires: ccache
%endif

Requires: bootloader-utils >= 0.4.24-alt1
Requires: module-init-tools >= 3.1
Requires: mkinitrd >= 1:2.9.9-alt1
Requires: startup >= 0.8.3-alt1

Provides: kernel = %kversion

Prereq: coreutils
Prereq: module-init-tools >= 3.1
Prereq: mkinitrd >= 1:2.9.9-alt1

%description
This package contains the Linux kernel that is used to boot and run
your system.

Most hardware drivers for this kernel are built as modules.  Some of
these drivers are built separately from the kernel; they are available
in separate packages (kernel-modules-*-%flavour).

The "un" variant of kernel packages is a low latency desktop oriented
2.6.x kernel which should support wide range of hardware,
but it is not 'official' ALT Linux kernel and you can use it for you
own risk.

%package -n kernel-image-domU-%flavour
Summary: Uncompressed linux kernel for XEN domU boot
Group: System/Kernel and hardware
Prereq: coreutils
Prereq: module-init-tools >= 3.1

%description -n kernel-image-domU-%flavour
Most XEN virtualization system versions can not boot lzma-compressed
kernel images. This is an optional package with uncompressed linux
kernel image for this special case. If you do not know what is it XEN
it seems that you do not need this package.


%package -n kernel-modules-drm-%flavour
Summary: The Direct Rendering Infrastructure modules
Group: System/Kernel and hardware
Provides:  kernel-modules-drm-%kversion-%flavour-%krelease = %version-%release
Conflicts: kernel-modules-drm-%kversion-%flavour-%krelease < %version-%release
Conflicts: kernel-modules-drm-%kversion-%flavour-%krelease > %version-%release
Prereq: coreutils
Prereq: module-init-tools >= 3.1
Prereq: %name = %version-%release
Requires(postun): %name = %version-%release

%description -n kernel-modules-drm-%flavour
The Direct Rendering Infrastructure, also known as the DRI, is a framework
for allowing direct access to graphics hardware in a safe and efficient
manner.  It includes changes to the X server, to several client libraries,
and to the kernel.  The first major use for the DRI is to create fast
OpenGL implementations.

These are modules for your ALT Linux system

%package -n kernel-modules-drm-nouveau-%flavour
Summary: The Direct Rendering Infrastructure modules for NVIDIA cards
Group: System/Kernel and hardware
Provides:  kernel-modules-drm-nouveau-%kversion-%flavour-%krelease = %version-%release
Conflicts: kernel-modules-drm-nouveau-%kversion-%flavour-%krelease < %version-%release
Conflicts: kernel-modules-drm-nouveau-%kversion-%flavour-%krelease > %version-%release
Requires: kernel-modules-drm-%kversion-%flavour-%krelease = %version-%release
Prereq: coreutils
Prereq: module-init-tools >= 3.1
Prereq: %name = %version-%release
Requires(postun): %name = %version-%release

%description -n kernel-modules-drm-nouveau-%flavour
The Direct Rendering Infrastructure, also known as the DRI, is a framework
for allowing direct access to graphics hardware in a safe and efficient
manner.  It includes changes to the X server, to several client libraries,
and to the kernel.  The first major use for the DRI is to create fast
OpenGL implementations.

These are modules for your ALT Linux system

%package -n kernel-modules-drm-radeon-%flavour
Summary: The Direct Rendering Infrastructure modules for ATI cards
Group: System/Kernel and hardware
Provides:  kernel-modules-drm-radeon-%kversion-%flavour-%krelease = %version-%release
Conflicts: kernel-modules-drm-radeon-%kversion-%flavour-%krelease < %version-%release
Conflicts: kernel-modules-drm-radeon-%kversion-%flavour-%krelease > %version-%release
Requires: kernel-modules-drm-%kversion-%flavour-%krelease = %version-%release
Prereq: coreutils
Prereq: module-init-tools >= 3.1
Prereq: %name = %version-%release
Requires(postun): %name = %version-%release

%description -n kernel-modules-drm-radeon-%flavour
The Direct Rendering Infrastructure, also known as the DRI, is a framework
for allowing direct access to graphics hardware in a safe and efficient
manner.  It includes changes to the X server, to several client libraries,
and to the kernel.  The first major use for the DRI is to create fast
OpenGL implementations.

These are modules for your ALT Linux system

%package -n kernel-modules-ide-%flavour
Summary: IDE  driver modules (obsolete by PATA)
Group: System/Kernel and hardware
Provides:  kernel-modules-ide-%kversion-%flavour-%krelease = %version-%release
Conflicts: kernel-modules-ide-%kversion-%flavour-%krelease < %version-%release
Conflicts: kernel-modules-ide-%kversion-%flavour-%krelease > %version-%release
Prereq: coreutils
Prereq: module-init-tools >= 3.1
Prereq: %name = %version-%release
Requires(postun): %name = %version-%release

%description -n kernel-modules-ide-%flavour
This package contains  IDE driver modules for the Linux kernel
package %name-%version-%release.

These drivers are declared obsolete by the kernel maintainers; PATA
drivers should be used instead.  However, the older IDE drivers may be
still useful for some hardware, if the corresponding PATA drivers do
not work well.

Install this package only if you really need it.

%package -n kernel-modules-kvm-%flavour
Summary: Linux KVM (Kernel Virtual Machine) modules
Group: System/Kernel and hardware
Provides:  kernel-modules-kvm-%kversion-%flavour-%krelease = %version-%release
Conflicts: kernel-modules-kvm-%kversion-%flavour-%krelease < %version-%release
Conflicts: kernel-modules-kvm-%kversion-%flavour-%krelease > %version-%release
Prereq: coreutils
Prereq: module-init-tools >= 3.1
Prereq: %name = %version-%release
Requires(postun): %name = %version-%release

%description -n kernel-modules-kvm-%flavour
Linux kernel module for Kernel Virtual Machine virtualization
environment.


%package -n kernel-modules-v4l-%flavour
Summary: Video4Linux driver modules (obsolete)
Group: System/Kernel and hardware
Provides:  kernel-modules-v4l-%kversion-%flavour-%krelease = %version-%release
Conflicts: kernel-modules-v4l-%kversion-%flavour-%krelease < %version-%release
Conflicts: kernel-modules-v4l-%kversion-%flavour-%krelease > %version-%release
Provides:  kernel-modules-uvcvideo-%kversion-%flavour-%krelease = %version-%release
Provides:  kernel-modules-gspca-%kversion-%flavour-%krelease = %version-%release
Provides:  kernel-modules-lirc-%kversion-%flavour-%krelease = %version-%release
Provides:  kernel-modules-lirc-%flavour = %version-%release
Prereq: coreutils
Prereq: module-init-tools >= 3.1
Prereq: %name = %version-%release
Requires(postun): %name = %version-%release

%description -n kernel-modules-v4l-%flavour
Video for linux drivers

%package -n kernel-modules-staging-%flavour
Summary:  Kernel modules under development
Group: System/Kernel and hardware
Provides:  kernel-modules-staging-%kversion-%flavour-%krelease = %version-%release
Conflicts: kernel-modules-staging-%kversion-%flavour-%krelease < %version-%release
Conflicts: kernel-modules-staging-%kversion-%flavour-%krelease > %version-%release
Requires: kernel-modules-drm-%kversion-%flavour-%krelease = %version-%release
Requires: kernel-modules-v4l-%kversion-%flavour-%krelease = %version-%release
Prereq: coreutils
Prereq: module-init-tools >= 3.1
Prereq: %name = %version-%release
Requires(postun): %name = %version-%release

%description -n kernel-modules-staging-%flavour
Drivers and filesystems that are not ready to be merged into the main
portion of the Linux kernel tree at this point in time for various
technical reasons.

%package -n kernel-headers-%flavour
Summary: Header files for the Linux kernel
Group: Development/Kernel
Requires: kernel-headers-common >= 1.1.5
Provides: kernel-headers = %version
#Provides: kernel-headers-%base_flavour = %version-%release

%description -n kernel-headers-%flavour
This package makes Linux kernel headers corresponding to the Linux
kernel package %name-%version-%release available for building
userspace programs (if this version of headers is selected by
adjust_kernel_headers).

Since Linux 2.6.18 the kernel build system supports creation of
sanitized kernel headers for use in userspace (by deleting headers
which are not usable in userspace and removing #ifdef __KERNEL__
blocks from installed headers).  This package contains sanitized
headers instead of raw kernel headers which were present in some
previous versions of similar packages.

If possible, try to use glibc-kernheaders instead of this package.

%package -n kernel-headers-modules-%flavour
Summary: Headers and other files needed for building kernel modules
Group: Development/Kernel
Requires: gcc%kgcc_version
Requires: libelf-devel

%description -n kernel-headers-modules-%flavour
This package contains header files, Makefiles and other parts of the
Linux kernel build system which are needed to build kernel modules for
the Linux kernel package %name-%version-%release.

If you need to compile a third-party kernel module for the Linux
kernel package %name-%version-%release, install this package
and specify %kbuild_dir as the kernel source
directory.

%package -n kernel-doc-%base_flavour
Summary: Linux kernel %kversion-%base_flavour documentation
Group: System/Kernel and hardware

%description -n kernel-doc-%base_flavour
This package contains documentation files for ALT Linux kernel packages:
 * kernel-image-%base_flavour-up-%kversion-%krelease
 * kernel-image-%base_flavour-smp-%kversion-%krelease

The documentation files contained in this package may be different
from the similar files in upstream kernel distributions, because some
patches applied to the corresponding kernel packages may change things
in the kernel and update the documentation to reflect these changes.

%ifarch %ix86 x86_64
%define Image bzImage
%endif

%ifarch mipsel
%define ImageTarget vmlinux.bin
%define ImagePath arch/%base_arch/boot/vmlinux.bin
%endif

%prep
%setup

# this file should be usable both with make and sh (for broken modules
# which do not use the kernel makefile system)
echo 'export GCC_VERSION=%kgcc_version' > gcc_version.inc

subst 's/EXTRAVERSION[[:space:]]*=.*/EXTRAVERSION = %kernel_extra_version-%flavour-%krelease/g' Makefile
subst 's/CC.*$(CROSS_COMPILE)gcc/CC         := $(shell echo $${GCC_USE_CCACHE:+ccache}) gcc-%kgcc_version/g' Makefile

# get rid of unwanted files resulting from patch fuzz
find . -name "*.orig" -delete -or -name "*~" -delete

%build
export ARCH=%base_arch
[ "%__nprocs" -gt "%nprocs" ] || export NPROCS=%nprocs
KernelVer=%kversion-%flavour-%krelease

echo "Building Kernel $KernelVer"

%make_build mrproper

cp -vf config-%_target_cpu .config

%make_build oldconfig
#%make_build include/linux/version.h
%make_build %ImageTarget
%make_build modules
%make_build dtbs

echo "Kernel built $KernelVer"

%if_enabled docs
# psdocs, pdfdocs don't work yet
%make_build htmldocs
%endif

%install
export ARCH=%base_arch
KernelVer=%kversion-%flavour-%krelease

install -Dp -m644 System.map %buildroot/boot/System.map-$KernelVer
install -Dp -m644 %ImagePath  %buildroot/boot/vmlinuz-$KernelVer
install -Dp -m644 vmlinux %buildroot/boot/vmlinux-$KernelVer
install -Dp -m644 .config %buildroot/boot/config-$KernelVer

make modules_install INSTALL_MOD_PATH=%buildroot INSTALL_FW_PATH=%buildroot/lib/firmware/$KernelVer

mkdir -p %buildroot/lib/devicetree/$KernelVer
find arch/%base_arch/boot/dts -type f -name \*.dtb |xargs -iz install -pm0644 z %buildroot/lib/devicetree/$KernelVer

mkdir -p %buildroot%kbuild_dir/arch/x86
install -d %buildroot%kbuild_dir
cp -a include %buildroot%kbuild_dir/include
cp -a arch/x86/include %buildroot%kbuild_dir/arch/x86


# drivers-headers install
install -d %buildroot%kbuild_dir/drivers/scsi
install -d %buildroot%kbuild_dir/drivers/md
install -d %buildroot%kbuild_dir/drivers/usb/core
install -d %buildroot%kbuild_dir/drivers/net/wireless
install -d %buildroot%kbuild_dir/net/mac80211
install -d %buildroot%kbuild_dir/kernel
install -d %buildroot%kbuild_dir/lib
cp -a drivers/scsi/{{scsi,scsi_typedefs}.h,scsi_module.c} \
	%buildroot%kbuild_dir/drivers/scsi/
cp -a drivers/md/dm*.h \
	%buildroot%kbuild_dir/drivers/md/
cp -a drivers/usb/core/*.h \
	%buildroot%kbuild_dir/drivers/usb/core/
cp -a drivers/net/wireless/Kconfig \
	%buildroot%kbuild_dir/drivers/net/wireless/
cp -a lib/hexdump.c %buildroot%kbuild_dir/lib/
cp -a kernel/workqueue.c %buildroot%kbuild_dir/kernel/
cp -a net/mac80211/ieee80211_i.h \
	%buildroot%kbuild_dir/net/mac80211/
cp -a net/mac80211/sta_info.h \
	%buildroot%kbuild_dir/net/mac80211/

# Install files required for building external modules (in addition to headers)
KbuildFiles="
	Makefile
	Module.symvers
	arch/%base_arch/Makefile
%ifarch %ix86 x86_64
	scripts/gcc-x86_*-has-stack-protector.sh
	arch/x86/Makefile_32
	arch/x86/Makefile_32.cpu
%endif
%ifarch x86_64
	arch/x86/Makefile_64
%endif

	scripts/pnmtologo
	scripts/mod/modpost
	scripts/mkmakefile
	scripts/mkversion
	scripts/link-vmlinux.sh
	scripts/mod/mk_elfconfig
	scripts/kconfig/conf
	scripts/mkcompile_h
	scripts/makelst
	scripts/Makefile.*
	scripts/Makefile
	scripts/Kbuild.include
	scripts/kallsyms
	scripts/genksyms/genksyms
	scripts/basic/fixdep
	scripts/basic/hash
	scripts/extract-ikconfig
	scripts/conmakehash
	scripts/checkversion.pl
	scripts/checkincludes.pl
	scripts/checkconfig.pl
	scripts/bin2c
	scripts/gcc-version.sh
	scripts/gcc-goto.sh
	scripts/recordmcount.pl
	scripts/recordmcount.h
	scripts/recordmcount.c
	scripts/recordmcount
	scripts/module-common.lds
	scripts/depmod.sh
	scripts/gcc-plugins/*.so
	scripts/ld-version.sh
	tools/objtool/objtool


	.config
	.kernelrelease
	gcc_version.inc
	System.map
"
for f in $KbuildFiles; do
	[ -e "$f" ] || continue
	[ -x "$f" ] && mode=755 || mode=644
	install -Dp -m$mode "$f" %buildroot%kbuild_dir/"$f"
done

# Fix symlinks to kernel sources in /lib/modules
rm -f %buildroot%modules_dir/{build,source}
ln -s %kbuild_dir %buildroot%modules_dir/build

# Provide kbuild directory with old name (without %%krelease)
ln -s "$(relative %kbuild_dir %old_kbuild_dir)" %buildroot%old_kbuild_dir

# Provide kernel headers for userspace
make headers_install INSTALL_HDR_PATH=%buildroot%kheaders_dir

#provide symlink to autoconf.h for back compat
pushd %buildroot%old_kbuild_dir/include/linux
ln -s ../generated/autoconf.h
ln -s ../generated/utsrelease.h
ln -s ../generated/uapi/linux/version.h
popd

# remove *.bin files
rm -f %buildroot%modules_dir/modules.{alias,dep,symbols,builtin}.bin
touch %buildroot%modules_dir/modules.{alias,dep,symbols,builtin}.bin

# install documentation
%if_enabled docs
install -d %buildroot%_docdir/kernel-doc-%base_flavour-%version/
cp -a Documentation/* %buildroot%_docdir/kernel-doc-%base_flavour-%version/
find %buildroot%_docdir/kernel-doc-%base_flavour-%version/DocBook \
	-maxdepth 1 -type f -not -name '*.html' -delete
%endif # if_enabled docs

%ifarch mipsel
	install -d %buildroot%kbuild_dir/arch
	cp -a arch/%base_arch %buildroot%kbuild_dir/arch
	find %buildroot%kbuild_dir/arch/%base_arch -name "*.o" -delete
	find %buildroot%kbuild_dir/arch/%base_arch -name ".*.cmd" -delete
	rm -f %buildroot%kbuild_dir/arch/%base_arch/boot/dts/include/dt-bindings
	rm -rf %buildroot%kbuild_dir/arch/x86
%endif

%check
KernelVer=%kversion-%flavour-%krelease
mkdir test
cd test
gcc -static -xc -o init - <<EOF
#include <unistd.h>
#include <sys/reboot.h>
int main()
{
        write( STDERR_FILENO, "Boot successfull!\n", 18);
        reboot( RB_POWER_OFF  );
        pause();
}
EOF
echo "init" | cpio -H newc -o | gzip > initrd.img
timeout 600 qemu -no-kvm -kernel %buildroot/boot/vmlinuz-$KernelVer -nographic -append console=ttyS0 -initrd initrd.img > boot.log
grep -q 'reboot: Power down' boot.log || ( cat boot.log && false )

%files
/boot/vmlinuz-%kversion-%flavour-%krelease
/boot/System.map-%kversion-%flavour-%krelease
/boot/config-%kversion-%flavour-%krelease
/lib/devicetree/%kversion-%flavour-%krelease

%modules_dir
%exclude %modules_dir/build
%exclude %modules_dir/kernel/drivers/gpu/drm

%if 0
%exclude %modules_dir/kernel/arch/x86/kvm
%exclude %modules_dir/kernel/drivers/media/
%exclude %modules_dir/kernel/drivers/staging/
%exclude %modules_dir/kernel/drivers/ide/
/lib/firmware/*
%endif

%ghost %modules_dir/modules.alias.bin
%ghost %modules_dir/modules.dep.bin
%ghost %modules_dir/modules.symbols.bin
%ghost %modules_dir/modules.builtin.bin

%files -n kernel-image-domU-%flavour
/boot/vmlinux-%kversion-%flavour-%krelease

%files -n kernel-headers-%flavour
%kheaders_dir

%files -n kernel-headers-modules-%flavour
%kbuild_dir
%old_kbuild_dir
%dir %modules_dir
%modules_dir/build

%if_enabled docs
%files -n kernel-doc-%base_flavour
%doc %_docdir/kernel-doc-%base_flavour-%version
%endif


%if 0
%files -n kernel-modules-kvm-%flavour
%modules_dir/kernel/arch/x86/kvm
%endif

%files -n kernel-modules-drm-%flavour
%modules_dir/kernel/drivers/gpu/drm

%if 0
%exclude %modules_dir/kernel/drivers/gpu/drm/nouveau
%exclude %modules_dir/kernel/drivers/gpu/drm/radeon

/lib/firmware/%kversion-%flavour-%krelease/radeon
%endif

%if 0

%files -n kernel-modules-drm-nouveau-%flavour
%modules_dir/kernel/drivers/gpu/drm/nouveau

%files -n kernel-modules-drm-radeon-%flavour
%modules_dir/kernel/drivers/gpu/drm/radeon
%files -n kernel-modules-ide-%flavour
%modules_dir/kernel/drivers/ide/

%files -n kernel-modules-v4l-%flavour
%modules_dir/kernel/drivers/media/
%modules_dir/kernel/drivers/staging/media/lirc/

%files -n kernel-modules-staging-%flavour
%modules_dir/kernel/drivers/staging/
%exclude %modules_dir/kernel/drivers/staging/media/lirc/

%endif

%changelog
* Wed May 13 2020 Ivan A. Melnikov <iv@altlinux.org> 4.4.215-alt4
- kernel-modules: package more scripts/

* Wed Mar 18 2020 Ivan A. Melnikov <iv@altlinux.org> 4.4.215-alt3
- Fix BMC initialization on older Tavolga boards.

* Thu Mar 12 2020 Alexey Sheplyakov <asheplyakov@altlinux.org> 4.4.215-alt2
  - Handle broken memory info in FDT so the kernel can boot out of the box
    on BFK boards and older revisions of Tavolga terminals.

* Tue Mar 10 2020  Alexey Sheplyakov <asheplyakov@altlinux.org> 4.4.215-alt1
  - Merged with linux-stable v4.4.215
  - Revert to vmlinux.bin (uncompressed) kernel images so older hardware
    can boot without reconfiguring uBoot

* Fri Feb 21 2020 Alexey Sheplyakov <asheplyakov@altlinux.org> 4.4.214-alt1
  - Merged with linux-stable v4.4.214

* Thu Jan 16 2020 Alexey Sheplyakov <asheplyakov@altlinux.org> 4.4.210-alt1
  - Merged with linux-stable v4.4.210

* Mon Jan 13 2020 Alexey Sheplyakov <asheplyakov@altlinux.org> 4.4.207-alt1
  - Merged with linux-stable v4.4.207

* Thu Jan 09 2020 Alexey Sheplyakov <asheplyakov@altlinux.org> 4.4.206-alt1
  - Merged with linux-stable 4.4.206
  - Reverted commit b4ca3857b8e7bc87 which breaks sm750 framebuffer
  - Config changes:
    + Enabled cifs POSIX extensions
    + Disabled KGDB (local user can easily gain root)
    + Enabled jffs2 filesystem

* Fri Nov 22 2019 Alexey Sheplyakov <asheplyakov@altlinux.org> 4.4.174-alt1
- Initial build
