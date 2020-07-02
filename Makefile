root_dir := $(PWD) 
src_dir := $(PWD)/src
syn_dir := $(PWD)/syn
inc_dir := $(PWD)/include
sim_dir := $(PWD)/sim
bld_dir := $(PWD)/build
FSDB_DEF :=
ifeq ($(FSDB),1)
FSDB_DEF := +FSDB
endif

$(bld_dir):
	mkdir -p $(bld_dir)
$(syn_dir):
	mkdir -p $(syn_dir)
	
#RTL Simulation
rtl_conv0: | $(bld_dir)
	cd $(bld_dir); \
	ncverilog $(sim_dir)/top_tb.v \
	+incdir+$(src_dir)+$(inc_dir)+$(sim_dir) \
	+define+conv0$(FSDB_DEF) \
	+access+r
	
rtl_pool1: | $(bld_dir)
	cd $(bld_dir); \
	ncverilog $(sim_dir)/top_tb.v \
	+incdir+$(src_dir)+$(inc_dir)+$(sim_dir) \
	+define+pool1$(FSDB_DEF) \
	+access+r

rtl_full: | $(bld_dir)
	cd $(bld_dir); \
	ncverilog $(sim_dir)/top_tb.v \
	+incdir+$(src_dir)+$(inc_dir)+$(sim_dir) \
	+define+full$(FSDB_DEF) \
	+access+r	
	
#Post-Synthesis simulation
syn_full: | $(bld_dir)
	cd $(bld_dir); \
	ncverilog $(sim_dir)/top_tb.v \
	+incdir+$(syn_dir)+$(inc_dir)+$(sim_dir) \
	+define+SYN+full$(FSDB_DEF) \
	+access+r
	
#Utilities
nWave: | $(bld_dir)
	cd $(bld_dir); \
	nWave top.fsdb &

superlint: | $(bld_dir)
	cd $(bld_dir); \
	jg -superlint ../script/superlint.tcl &
	
synthesize: | $(bld_dir) $(syn_dir)
	cd $(bld_dir); \
	dc_shell -f ../script/synthesis.tcl

tar: clean
	STUDENTID=$$(basename $(PWD)); \
	cd ..;\
	tar cvf $$STUDENTID.tar $$STUDENTID
	
.PHONY: clean
	
clean:
	rm -rf $(bld_dir);

	
