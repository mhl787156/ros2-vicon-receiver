#!/bin/sh

set -e

echo "Installing shared libraries, please wait"
sudo cp DataStreamSDK_1.11/* /usr/lib/
sudo chmod -R 0755  /usr/lib

export VICON_LIBRARY=/usr/lib/
echo "."
sudo ldconfig
echo "."
echo "Installlation finished"

