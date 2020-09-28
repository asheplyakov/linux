dtb-$(CONFIG_DTB_BAIKAL_BFK3)	+= baikal_bfk3.dtb


dtb-$(CONFIG_DT_NONE)	+= \
						baikal_bfk3.dtb	\

obj-y				+= $(patsubst %.dtb, %.dtb.o, $(dtb-y))

# Force kbuild to make empty built-in.o if necessary
obj-				+= dummy.o

always				:= $(dtb-y)
clean-files			:= *.dtb *.dtb.S
