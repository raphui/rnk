#!/bin/bash

FILE=utils/symbols.c
BAK=utils/symbols.c.bak

make()
{

	LIST=$(nm kernel.elf --print-size | grep  " T " \
		| sed -e 's/^/\t{/g' -e 's/$/"},/g' -e 's/\sT\s/, "/g' -e 's/{/{0x/g' -e 's/\s/, /2' -e 's/,,/, 00000000, /g' \
		| awk '{$2="0x"$2}1')

	awk -v l="$LIST" '/Generate/ { print; print l; next }1' $FILE > $BAK
	mv $BAK $FILE
}

clean()
{
	sed -i '/},/d' $FILE
}

case "$1" in
	make)
		make	
		;;
	clean)
		clean
		;;
	*)
		echo "Usage: $0 {make|clean}"
		exit 1
esac

exit $?
