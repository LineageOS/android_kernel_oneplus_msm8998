#!/bin/bash
cd ~
sudo apt-get update && sudo apt-get upgrade -y
sudo apt install -y bc bison build-essential ccache curl flex g++-multilib gcc-multilib git gnupg gperf lib32ncurses5-dev lib32readline-dev lib32z1-dev liblz4-tool libncurses5 libncurses-dev libsdl1.2-dev libssl-dev libxml2 libxml2-utils lzop pngcrush rsync schedtool squashfs-tools xsltproc zip zlib1g-dev
#install clang
git clone https://github.com/kpzhao/proton-clang clang-proton
