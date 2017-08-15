GCC_VERSION      := $(shell $(CONFIG_SHELL) $(srctree)/scripts/gcc-version.sh $(CROSS_COMPILE)gcc)
ccflags-y += -Idrivers/media/video/msm
ccflags-y += -Idrivers/media/platform/msm/camera_v2_oem/common
ccflags-y += -Idrivers/media/platform/msm/camera_v2_oem
ccflags-y += -Idrivers/media/platform/msm/camera_v2_oem/pproc/cpp
ccflags-y += -Idrivers/media/platform/msm/camera_v2_oem/msm_buf_mgr/

obj-$(CONFIG_MSM_FD) += msm_fd_dev.o msm_fd_hw.o
