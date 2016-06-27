#!/bin/bash

if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root" 1>&2
   exit 1
fi

sudo apt-get install -y python make

echo "Building Library..."

mkdir -p build/lib &> /dev/null
mkdir -p build/clients/ros &> /dev/null
mkdir build/clients/python &> /dev/null
mkdir build/clients/c++ &> /dev/null

make > /dev/null
cp lib/* build/lib

echo "Adding User to correct groups"
sudo usermod -a -G dialout $USER

read -p "Make sure the RFID reader is unplugged and then press enter" dummy
#unplugged="$(ls /dev | grep "ttyUSB")"
unplugged="$(ls /dev | grep "ttyACM")"
read -p "Now plug the RFID reader in and then press enter" dummy
#plugged="$(ls /dev | grep "ttyUSB")"
plugged="$(ls /dev | grep "ttyACM")"

fileList="$(diff <(echo "$unplugged") <(echo "$plugged"))"

array=(${fileList// / })

for element in "${array[@]}"
do
	#if [[ $element == "ttyUSB"* ]]
	if [[ $element == "ttyACM"* ]]
	then
		file="$element"
	fi
done


if [ -z "$file" ]
then
  echo "RFID reader not detected as usb device !!!!"
  exit 1 # terminate and indicate error
fi

echo RFID detected at: $file
echo "Persistant symlink created at /dev/rfid for this device"

prod="$(udevadm info -a -n /dev/$file | grep '{idProduct}' | head -n1 | tr -d '[[:space:]]')"
vend="$(udevadm info -a -n /dev/$file | grep '{idVendor}' | head -n1 | tr -d '[[:space:]]')"
seri="$(udevadm info -a -n /dev/$file | grep '{serial}' | head -n1 | tr -d '[[:space:]]')"

echo "SUBSYSTEM==\"tty\", $vend, $prod, $seri, SYMLINK+=\"rfid\"" > /etc/udev/rules.d/64-persistant-rfid.rules

sudo udevadm control --reload-rules

echo "Settings updated, unplug and replug rfid device for /dev/rfid to appear"

echo "Installing RFID libraries"

ln -s libmercuryrfid.so.1.0 build/lib/libmercuryrfid.so.1 &> /dev/null

cp build/lib/libltkc.so.1 /usr/lib/
cp build/lib/libltkctm.so.1 /usr/lib/
cp build/lib/libmercuryapi.so.1 /usr/lib/
cp build/lib/libmercuryrfid.so.1.0 /usr/lib/
ln -s libmercuryrfid.so.1.0 /usr/lib/libmercuryrfid.so.1

echo "Installing Client Modules"

#Python
mkdir build/clients/python &> /dev/null
cp src/clients/python/rfid.py build/clients/python/rfid.py
chmod +x build/clients/python/rfid.py
cp build/clients/python/rfid.py /usr/lib/python2.7/rfid.py

#ROS
mkdir build/clients/ros &> /dev/null
cp -r src/clients/ros/ build/clients/ros

#C++
ln -s libmercuryrfid-c.so.1.0 build/clients/c++/libmercuryrfid-c.so.1
cp build/clients/c++/libmercuryrfid-c.so.1.0 /usr/lib
ln -s libmercuryrfid-c.so.1.0 /usr/lib/libmercuryrfid-c.so.1

echo "Finishing Installation"

ldconfig

echo "Finished Installing"
