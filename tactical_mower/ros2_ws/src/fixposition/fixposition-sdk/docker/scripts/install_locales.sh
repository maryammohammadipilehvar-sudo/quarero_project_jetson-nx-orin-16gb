#!/bin/bash
########################################################################################################################
# ___    ___
# \  \  /  /
#  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
#  /  /\  \   License: see the LICENSE file
# /__/  \__\
#
########################################################################################################################
#
# Set timezone, sane language and locale
#
########################################################################################################################
set -eEu

set -e
export DEBIAN_FRONTEND=noninteractive
apt-get -y update
apt-get -y --with-new-pkgs upgrade
apt-get -y install locales sudo
apt-get clean
ln -snf /usr/share/zoneinfo/$TZ /etc/localtime
echo $TZ > /etc/timezone
sed -i '/en_GB.UTF-8/s/^# //g' /etc/locale.gen
locale-gen

########################################################################################################################
