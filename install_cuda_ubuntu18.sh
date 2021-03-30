#install CUDA (IMPORTANT: Do not have any form for nvidia drivers installed except for nouveau, otherwise it will break your package manager)

# add cuda paths
export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

if [ ! -f /usr/local/cuda/bin/nvcc ]; then
	cd /tmp/
	echo "Cuda not found! installing cuda 10.2"
	# nvidia sucks, so you can't use the new versions (11.1-2) since cuda crashes 
	# half of the time or throws errors at you when using pytorch. their own pytorch 
	# containers doesn't even use that version :) 
	sleep 5
	# sudo apt-get install -y linux-headers-$(uname -r)
	# wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin
	# sudo mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600
	# wget https://developer.download.nvidia.com/compute/cuda/11.2.1/local_installers/cuda-repo-ubuntu1804-11-2-local_11.2.1-460.32.03-1_amd64.deb
	# sudo dpkg -i cuda-repo-ubuntu1804-11-2-local_11.2.1-460.32.03-1_amd64.deb
	# sudo apt-key add /var/cuda-repo-ubuntu1804-11-2-local/7fa2af80.pub
	# sudo apt-get update
	# sudo apt-get -y install cuda
	wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin
	sudo mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600
	wget https://developer.download.nvidia.com/compute/cuda/10.2/Prod/local_installers/cuda-repo-ubuntu1804-10-2-local-10.2.89-440.33.01_1.0-1_amd64.deb
	sudo dpkg -i cuda-repo-ubuntu1804-10-2-local-10.2.89-440.33.01_1.0-1_amd64.deb
	sudo apt-key add /var/cuda-repo-10-2-local-10.2.89-440.33.01/7fa2af80.pub
	sudo apt-get update
	sudo apt-get -y install cuda-10-2
	echo ""
	echo "please reboot and proceed with readme-jetson.md"
	echo "export PATH=/usr/local/cuda/bin:${PATH}" >> ~/.bashrc
	echo "export LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}" >> ~/.bashrc
else
	echo "Cuda found:"
	echo ""
	echo $($(which nvcc) --version)
	echo ""
	echo "If cuda'<'10.2, then please uninstall and startover. Please uninstall nvidia and install nouveau before proceeding. Once uninstalled run this script again"
	echo ""
	echo "Proceed with readme-jason.md"
fi



# continue with readme-jetson.md
