#install CUDA (IMPORTANT: Do not have any form for nvidia drivers installed except for nouveau, otherwise it will break your package manager)

if [ ! -f /usr/local/cuda/bin/nvcc ]; then
	echo "Cuda not found! installing cuda 11.2"
	sleep 5
	sudo apt-get install -y linux-headers-$(uname -r)
	wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin
	sudo mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600
	wget https://developer.download.nvidia.com/compute/cuda/11.2.1/local_installers/cuda-repo-ubuntu1804-11-2-local_11.2.1-460.32.03-1_amd64.deb
	sudo dpkg -i cuda-repo-ubuntu1804-11-2-local_11.2.1-460.32.03-1_amd64.deb
	sudo apt-key add /var/cuda-repo-ubuntu1804-11-2-local/7fa2af80.pub
	sudo apt-get update
	sudo apt-get -y install cuda
	echo ""
	echo "please reboot and proceed with readme-jetson.md"
else
	echo "Cuda found:"
	echo ""
	echo $(/usr/local/cuda/bin/nvcc --version)
	echo ""
	echo "If cuda'<'10.2, then please uninstall and startover. Please uninstall nvidia and install nouveau before proceeding. Once uninstalled run this script again"
	echo ""
	echo "Proceed with readme-jason.md"
fi



# continue with readme-jetson.md
