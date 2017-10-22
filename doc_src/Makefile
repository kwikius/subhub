# aerflite doc makefile

this_dir_rst = $(wildcard *.rst)

objects_html = $(patsubst %.rst,../doc/%.html,$(this_dir_rst))

#---------------

all : $(objects_html)

$(objects_html) : ../doc/%.html : %.rst
	rst2html $< $@

