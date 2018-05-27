#!/bin/bash
address=0xfc00
echo "branchtable:"
while [ $address != "0x10000" ]
do
	echo $address >&2
	JUMP=register_unused
	if [ "$address" = "0xfc71" -o "$address" = "0xfc72" -o "$address" = "0xfe05" ];then
		JUMP="register_$address"
	fi
	echo "	.short     (($JUMP - branchtable)/2) /* $address */"
	address="`printf '0x%X' $((address + 1))|tr '[:upper:]' '[:lower:]'`"

done
