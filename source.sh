source install/setup.sh
source .venv/bin/activate

dir=$(pwd)
pkgs=$(ls -d install/*/)
python_path="/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:${dir}/.venv/lib/python3.10/site-packages"
for pkg_path in ${pkgs[@]}
do
	sub_directories="$(ls "$pkg_path")"
	echo $sub_directories
	if echo $sub_directories | grep -q "local"; then
		python_path="${python_path}:${dir}/${pkg_path}local/lib/python3.10/dist-packages"
	elif echo $sub_directories | grep -q "lib"; then
		python_path="${python_path}:${dir}/${pkg_path}lib/python3.10/site-packages"
	fi
done
export PYTHONPATH=$python_path
