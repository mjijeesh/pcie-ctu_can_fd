QUARTUS_PROJECT_NAME = db4cgx15_pcie_ctu_can_fd

QUARTUS_PROJECT_FILE = $(QUARTUS_PROJECT_NAME).qsf

SOURCE_FILES_FROM_QSF=$(shell sed -n 's/set_global_assignment -name [^ \t]*_FILE \([^ ]*\)\(\| -.*\)$$/\1/p' < $(QUARTUS_PROJECT_FILE) )

$(warning $(SOURCE_FILES_FROM_QSF))

all : $(QUARTUS_PROJECT_NAME).svf

output_files/$(QUARTUS_PROJECT_NAME).sof : $(QUARTUS_PROJECT_FILE) $(SOURCE_FILES_FROM_QSF)
	quartus_sh --flow compile $<
	@awk '/; File Name with User-Entered Path/,/^$$/' \
	    <output_files/$(QUARTUS_PROJECT_NAME).map.rpt | \
	    sed -n -e 's/^;[^;]*;[^;]*;[^;]*; \(\/[^; ]*\) *;.*$$/\1/p'  \
	    >$(QUARTUS_PROJECT_NAME).dep.tmp || true
	@sed -n -e 's#^\(.*\)$$#\1:#p' <$(QUARTUS_PROJECT_NAME).dep.tmp >$(QUARTUS_PROJECT_NAME).dep || true
	@sed -n -e 's#^\(.*\)$$#$@:\1#p' <$(QUARTUS_PROJECT_NAME).dep.tmp >>$(QUARTUS_PROJECT_NAME).dep || true
	@rm $(QUARTUS_PROJECT_NAME).dep.tmp


$(QUARTUS_PROJECT_NAME).svf : output_files/$(QUARTUS_PROJECT_NAME).sof
	quartus_cpf -c -q 12MHz -g 3.3 -n v $< $@

clean:
	rm -f *.rpt *.sopcinfo *.bak *.dep *.orig *.svf *.qws
	rm -rf output_files incremental_db .qsys_edit pcie_core greybox_tmp db


.PHONNY: clean all

-include $(QUARTUS_PROJECT_NAME).dep
