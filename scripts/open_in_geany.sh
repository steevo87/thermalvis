#!/bin/bash
# chmod 777 open_in_geany.sh

## This script will open all relevant project files in geany.
## It is designed to make it easier to search and edit the source files 
## in the project.

max_files=200

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "Script location is ${SCRIPT_DIR}"
echo "Searching for all source files.."

file_count=0

for f in $(find ${SCRIPT_DIR}/.. -name '*.h*' -or -name '*.c*' -or -name '*.sh' -or -name '*.txt');
	do file_count=$((file_count+1))
done

echo "Total of (${file_count}) files found."

if [[ "$file_count" -gt "$max_files" ]]; then
	echo "Too many files to safely open. Exiting script."
	exit 1
fi

for f in $(find ${SCRIPT_DIR}/.. -name '*.h*' -or -name '*.c*');
	do geany $f &
done
