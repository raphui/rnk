#!/bin/bash

for d in $APPS_BASE/tests/*; do
	cd $d
	make clean
	make
done
