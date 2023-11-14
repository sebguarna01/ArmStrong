#!/bin/zsh

# ################################# ENV VARS ################################# #
VENV_DIR="ros_venv"
PROJECT_DIR=$(pwd)

# ################################# FUNCTIONS ################################ #
setup_venv() {
    if [ ! -d "${VENV_DIR}" ]; then
        echo "CREATING PYTHON VIRTUAL ENVIRONMENT..."
        python3 -m venv "${VENV_DIR}"
        echo "DONE..."
    else
        echo "PYTHON VIRTUAL ENVIRONMENT ALREADY EXITS... SKIPPING"
    fi
    
}

install_reqs() {
    if [ -f "requirements.txt" ]; then
        echo "INSTALLING PYTHON REQUIREMENTS..."
        pip3 install -r requirements.txt -q
        echo "DONE..."
    else
        echo "REQUIREMENTS FILE NOT FOUND... SKIPPING"
    fi
}

catkin_init() {
    if [ ! -f src/CMakeLists.txt ]; then
        echo "RUN CATKIN_INIT_WORKSPACE..."
        catkin_init_workspace
        echo "DONE..."
    else
        echo "CATKIN_INIT FILE EXISTS... SKIPPING"
    fi
}

catkin_make_func() {
    if [ ! -d build ] && [ ! -d devel ]; then
        echo "RUN CATKIN_MAKE..."
        catkin_make
        echo "DONE..."
    else
        echo "CATKIN_MAKE DIRS EXIST... SKIPPING"
    fi
}

terminal_functions() {
    cd $PROJECT_DIR
    # SETUP PYTHON VENV
    setup_venv
    # SOURCE PYTHON VENV
    echo "SOURCING PYTHON VENV..."
    source ros_venv/bin/activate
    echo "DONE..."
    install_reqs

    echo "SOURCING GLOBAL ROS IN ${HOME}..."
    cd $HOME
    source /opt/ros/noetic/setup.zsh
    echo "DONE..."

    cd $PROJECT_DIR
    catkin_init
    catkin_make_func
    echo "SOURCING LOCAL ROS IN ${PROJECT_DIR}..."
    source "${PROJECT_DIR}/devel/setup.zsh"
    echo "DONE..."
}

# ################################### MAIN ################################### #
terminal_functions