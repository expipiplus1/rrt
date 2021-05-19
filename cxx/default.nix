{ rosOverlay ? builtins.fetchTarball {
  url =
    "https://github.com/lopsided98/nix-ros-overlay/archive/0d3aca6651c13bca684339a40f6f13d3f90066aa.tar.gz"; # refs/heads/master
  sha256 = "03wf835iw2clcms7k3panlvc985pca44bsy595c4mpphnvl3dw62";
}, pkgs ? import rosOverlay { } }:

with pkgs;
with rosPackages.noetic;

mkShell {
  nativeBuildInputs = [ gdb catch2 eigen stb rosbash xacro rosnode nav-msgs rviz pinta ];

  ROS_HOSTNAME = "localhost";
  ROS_MASTER_URI = "http://localhost:11311";
}
