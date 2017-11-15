#!/usr/bin/env bash

REPO_NAME=mvimpact_acquire
cd ${REPO_NAME}
PLATFORM=$(uname -i)
if [[ ${PLATFORM} == "x86_64" ]] ; then
    echo "Installing 64-bit version"
    cd x86-64
    sudo ./install_mvBlueFOX.sh
elif [[ ${PLATFORM} == "armv7l" ]]; then
    echo "Installing arm version"
    cd armv7l
    sudo ./install_mvBlueFOX_arm.sh
else
    echo "Unknown platform ${PLATFORM}"
fi
echo "Done."
