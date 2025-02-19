#!/usr/bin/env bash

# Original javascript implementation can be found at: http://www.zorc.breitbandkatze.de/crc.html
# http://www.ross.net/crc/download/crc_v3.txt
# http://wiki.osdev.org/CRC32

function reflect( )
{
	local -a crc=( "${!1}" )
	local bitnum=$2
	local startLSB=$3

	local i=0 
	local j=0
	local iw=0
	local jw=0
	local bit=0

	for (( k=0; $((k + startLSB)) < $(( bitnum - 1 - k )); k++ ))
	do
		iw=$(( 7 - ( (k + startLSB) >> 3 ) ))
		jw=$(( 1 << ( ( k + startLSB) & 7 ) ))
		i=$(( 7 - ( ( bitnum - 1 - k ) >> 3 ) ))
		j=$(( 1 << ( ( bitnum - 1 - k ) & 7 ) ))
		
		bit=$(( crc[iw] & jw ))
		if [ $(( crc[i] & j )) -ne 0 ]
		then
			crc[iw]=$(( crc[iw] | jw ))
		else
			crc[iw]=$(( crc[iw] & (0xff - jw) ))
		fi
		
		if [ $bit -ne 0 ]
		then
			crc[i]=$(( crc[i] | j ))
		else
			crc[i]=$(( crc[i] & (0xff - j) ))
		fi	
	done
	echo ${crc[@]}
}

function reflectbyte( )
{
	local char=$( printf '%d' \'"${1}" )
	local outbyte=0
	local i=0x01

	for (( j=0x80; j; j=$(( j>>1 )) ))
	do
		if [ $(( char & i )) -ne 0 ]
		then
			outbyte=$(( outbyte | j ))
		fi
		i=$(( i << 1 ))
	done
	echo "$outbyte"
}

function crc16_8005( ) 
{
	local order=16
	local char=""
	local -a polynom=( 0 0 0 0 0 0 128 5 )
	local -a mask=( 0 0 0 0 0 0 255 255 )
	local -a crc=( 0 0 0 0 0 0 0 0 0 )

	local input=$1
	local len=${#input}

	for (( i=0; i<len; i++ ))
	do
		char=${1:$i:1}
		char=$( reflectbyte $char )

		for (( j=0; j<8; j++ ))
		do
			bit=0
			if [ $(( crc[6] & 128 )) -ne 0 ]
			then
				bit=1
			fi

			if [ $(( char & 0x80 )) -ne 0 ]
			then
				bit=$(( bit ^ 1 ))
			fi
			char=$(( char << 1 ))
	
			for (( k=6; k<8; k++ ))
			do
				crc[$k]=$(( ( (crc[k] << 1) | (crc[k+1] >> 7) ) & mask[k] ))
				if [ $bit -ne 0 ]
				then
					crc[$k]=$(( crc[k] ^ polynom[k] ))
				fi
			done
		done
	done

	local -a new_crc=$( reflect crc[@] 16 0 )
	crc=( ${new_crc[@]} )

	local output=""

	for (( i=6; i<8; i++ ))
	do
		output+=$( printf "%02x" ${crc[i]} )
	done

	echo "output: $output"
}

crc16_8005 "$1"
