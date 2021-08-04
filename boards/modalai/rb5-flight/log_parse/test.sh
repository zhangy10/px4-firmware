#!/bin/bash

counter=1
valid=1
while [ $valid -eq 1 ]; do
rm -f 21_23_56.ulg
echo $counter
((counter++))
adb pull /home/linaro/log/2021-07-24/21_23_56.ulg
filesize=$(wc -c "21_23_56.ulg" | awk '{print $1}')
if [ $filesize -ne 58048512 ]; then
   valid=0
fi
done
