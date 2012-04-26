all: wmm

TARBALL = build/WMM2010_Linux.tar.gz
TARBALL_URL = http://www.ngdc.noaa.gov/geomag/WMM/data/WMM2010/WMM2010_Linux.tar.gz
SOURCE_DIR = build/src
MD5SUM_FILE = WMM2010_Linux.tar.gz.md5sum
UNPACK_CMD = mkdir -p ../$(SOURCE_DIR); tar xz -C ../$(SOURCE_DIR) -f
include $(shell rospack find mk)/download_unpack_build.mk

wmm: lib/libworld_magnetic_model.so

lib/libworld_magnetic_model.so: $(SOURCE_DIR)/unpacked
	cd build && cmake $(CMAKE_FLAGS) ..
ifneq ($(MAKE),)
	cd build && $(MAKE) $(ROS_PARALLEL_JOBS)
else
	cd build && make $(ROS_PARALLEL_JOBS)
endif
	mkdir -p include/world_magnetic_model && cp $(SOURCE_DIR)/*.h include/world_magnetic_model
	mkdir -p model && cp $(SOURCE_DIR)/*.COF model

clean:
	-rm -rf lib include model $(SOURCE_DIR)
wipe: clean
	-rm -rf build
