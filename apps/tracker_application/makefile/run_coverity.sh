#!/bin/bash     

rm -r html_output
rm -r idir
cov-build --dir idir make clean
cov-build --dir idir make -j
cov-analyze --dir idir --strip-path /home/bboulet/Documents/Documents/dev/lr1110/
mkdir html_output
cov-format-errors --dir idir --html-output ./html_output/
