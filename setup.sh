#!/usr/bin/env bash

set -e

export DEBIAN_FRONTEND=noninteractive

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get dist-upgrade -y

sudo apt-get install -yq \
    "ros-noetic-turtlebot3*" \
    bash-completion \
    black \
    git \
    ipython3 \
    locales \
    neovim \
    python3-{pip,numpy,scipy} \
    ros-noetic-desktop-full \
    ros-noetic-{dynamixel-sdk,slam-gmapping,dwa-local-planner} \
    tmux

pip3 install setuptools==58.2.0
{
    echo "source /usr/share/bash-completion/bash_completion"
    echo "source /opt/ros/noetic/setup.bash"
} >> /etc/bash.bashrc

mkdir -p ~/.config/nix ~/.nixpkgs
{
    echo allowed-users = root
    echo auto-optimise-store = true
    echo build-users-group = nixbld
    echo cores = 0
    echo experimental-features = nix-command flakes
    echo max-jobs = auto
    echo sandbox = false
} > ~/.config/nix/nix.conf
echo "{ allowUnfree = true; }" > ~/.nixpkgs/config.nix

sh <(curl -L https://nixos.org/nix/install) --daemon

export PATH=$HOME/.nix-profile/bin:/nix/var/nix/profiles/default/bin:$PATH
echo "export PATH=$HOME/.nix-profile/bin:$PATH" >> ~/.bashrc

nix profile install github:acristoffers/nvim
nix-collect-garbage -d
nix-store --verify --check-contents
nix store optimise
