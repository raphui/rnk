#!/bin/bash

FILE=utils/symbols.c
BAK=utils/symbols.c.bak

make()
{
	LIST=$(nm -S kernel.elf | awk '{line=$3; $1="0x"$1; $2="0x"$2; if(NF==4){line=$4}else{$2="0"}; if (line != "$d" && line != "$t"){printf("{%s, %s, \"%s\"},\n", $1, $2, line)}}')
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
