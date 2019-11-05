dtb-$(CONFIG_DTB_BAIKAL_BFK)	+= baikal_bfk.dtb
dtb-$(CONFIG_DTB_BAIKAL_BFK3)	+= baikal_bfk3.dtb baikal_bfk2.dtb
dtb-$(CONFIG_DTB_BAIKAL_BFKX)	+= baikal_bfkx.dtb
dtb-$(CONFIG_DTB_BAIKAL_FPGA)	+= baikal_fpga.dtb
dtb-$(CONFIG_DTB_BAIKAL_QEMU)	+= baikal_qemu.dtb
dtb-$(CONFIG_DTB_TPLATFORMS_SFBT1)	+= tplatforms_sfbt1.dtb
dtb-$(CONFIG_DTB_TPLATFORMS_MSBT2)	+= tplatforms_msbt2.dtb \
						tplatforms_xa1-msbt2.dtb
dtb-$(CONFIG_DTB_TPLATFORMS_MRBT1)	+= tplatforms_mrbt1.dtb \
						tplatforms_xa1-mrbt1.dtb
dtb-$(CONFIG_DTB_TPLATFORMS_MITX)	+= tplatforms_mitx.dtb
dtb-$(CONFIG_DTB_TPLATFORMS_RT1MB)	+= tplatforms_rt1mb.dtb
dtb-$(CONFIG_DTB_TPLATFORMS_TC_SFBT1)	+= tplatforms_tc-sfbt1.dtb
dtb-$(CONFIG_DTB_TPLATFORMS_TC_MSBT2)	+= tplatforms_tc-msbt2.dtb
dtb-$(CONFIG_DTB_TPLATFORMS_AXT1_SFBT1)	+= tplatforms_axt1-sfbt1.dtb
dtb-$(CONFIG_DTB_TPLATFORMS_AXT1_MSBT2)	+= tplatforms_axt1-msbt2.dtb
dtb-$(CONFIG_DTB_TPLATFORMS_BC3BT1_MRBT1)	+= tplatforms_bc3bt1-mrbt1.dtb
dtb-$(CONFIG_DTB_TPLATFORMS_CNC_SFBT1)	+= tplatforms_cnccu-sfbt1.dtb \
						tplatforms_cnccu3-sfbt1.dtb \
						tplatforms_cncin-sfbt1.dtb \
						tplatforms_cncin3-sfbt1.dtb \
						tplatforms_cncout-sfbt1.dtb \
						tplatforms_cncout3-sfbt1.dtb \
						tplatforms_cncttl-sfbt1.dtb \
						tplatforms_cncttl3-sfbt1.dtb \
						tplatforms_cncdac-sfbt1.dtb \
						tplatforms_cncdac3-sfbt1.dtb
dtb-$(CONFIG_DTB_TPLATFORMS_CNC_MSBT2)	+= tplatforms_cnccu-msbt2.dtb \
						tplatforms_cnccu3-msbt2.dtb \
						tplatforms_cncin-msbt2.dtb \
						tplatforms_cncin3-msbt2.dtb \
						tplatforms_cncout-msbt2.dtb \
						tplatforms_cncout3-msbt2.dtb \
						tplatforms_cncttl-msbt2.dtb \
						tplatforms_cncttl3-msbt2.dtb \
						tplatforms_cncdac-msbt2.dtb \
						tplatforms_cncdac3-msbt2.dtb

dtb-$(CONFIG_DT_NONE)	+= \
						baikal_bfk.dtb	\
						baikal_bfk3.dtb	\
						baikal_bfkx.dtb	\
						baikal_fpga.dtb	\
						baikal_qemu.dtb \
						tplatforms_sfbt1.dtb \
						tplatforms_msbt2.dtb \
						tplatforms_xa1-msbt2.dtb \
						tplatforms_mrbt1.dtb \
						tplatforms_xa1-mrbt1.dtb \
						tplatforms_mitx.dtb \
						tplatforms_rt1mb.dtb \
						tplatforms_tc-sfbt1.dtb \
						tplatforms_tc-msbt2.dtb \
						tplatforms_axt1-sfbt1.dtb \
						tplatforms_axt1-msbt2.dtb \
						tplatforms_bc3bt1-mrbt1.dtb \
						tplatforms_cnccu-sfbt1.dtb \
						tplatforms_cnccu3-sfbt1.dtb \
						tplatforms_cncin-sfbt1.dtb \
						tplatforms_cncin3-sfbt1.dtb \
						tplatforms_cncout-sfbt1.dtb \
						tplatforms_cncout3-sfbt1.dtb \
						tplatforms_cncttl-sfbt1.dtb \
						tplatforms_cncttl3-sfbt1.dtb \
						tplatforms_cncdac-sfbt1.dtb \
						tplatforms_cncdac3-sfbt1.dtb \
						tplatforms_cnccu-msbt2.dtb \
						tplatforms_cnccu3-msbt2.dtb \
						tplatforms_cncin-msbt2.dtb \
						tplatforms_cncin3-msbt2.dtb \
						tplatforms_cncout-msbt2.dtb \
						tplatforms_cncout3-msbt2.dtb \
						tplatforms_cncttl-msbt2.dtb \
						tplatforms_cncttl3-msbt2.dtb \
						tplatforms_cncdac-msbt2.dtb \
						tplatforms_cncdac3-msbt2.dtb

obj-y				+= $(patsubst %.dtb, %.dtb.o, $(dtb-y))

# Force kbuild to make empty built-in.o if necessary
obj-				+= dummy.o

always				:= $(dtb-y)
clean-files			:= *.dtb *.dtb.S
