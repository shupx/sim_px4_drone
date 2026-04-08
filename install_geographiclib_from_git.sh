#!/usr/bin/env bash
set -e

apt_install_if_missing() {
	local pkg="$1"
	if dpkg -s "$pkg" >/dev/null 2>&1; then
		echo "[pre_install] $pkg already installed, skip"
		return 0
	fi
	echo "[pre_install] installing $pkg"
	sudo apt-get install -y "$pkg"
}

echo ""
echo "----------- Installing GeographibLib ------------------"
if [ ! -d "/usr/share/GeographicLib/geoids" ]; then
    apt_install_if_missing "git"
    apt_install_if_missing "unzip"
    cd ~
    [ -d geographiclib_manual/ ] && rm -rf geographiclib_manual/
    git clone --depth 1 https://gitee.com/shu-peixuan/geographiclib_manual.git
    cd geographiclib_manual/
    sudo chmod +x manual_install_geographiclib.sh
    ./manual_install_geographiclib.sh
    cd ..
    rm -rf geographiclib_manual/
    # wget https://gitee.com/shu-peixuan/ros-install-command/raw/c9865c748045a0cce0173fcfcb95729784bd31e5/install_geographiclib_datasets.sh
    # sudo chmod a+x ./install_geographiclib_datasets.sh
    # sudo ./install_geographiclib_datasets.sh # this step takes some time
    # rm install_geographiclib_datasets.sh
else
    echo "Geographiclib already installed, skipping installation."
fi