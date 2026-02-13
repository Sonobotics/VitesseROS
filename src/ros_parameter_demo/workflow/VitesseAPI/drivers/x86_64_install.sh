#!/bin/bash
cd $(pwd)/linux_x86_64_FTDI
cp 99-ftdi.rules /etc/udev/rules.d/
tar xfvz libftd2xx-x86_64-1.4.27.tgz
cd release/build/
cp libftd2xx.* /usr/local/lib
chmod 0755 /usr/local/lib/libftd2xx.so.1.4.27
ln -sf /usr/local/lib/libftd2xx.so.1.4.27 /usr/local/lib/libftd2xx.so
cd ..
cp ftd2xx.h  /usr/local/include
cp WinTypes.h  /usr/local/include
ldconfig -v
exit