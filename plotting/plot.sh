#! /bin/bash

rp=$(realpath ${0})
ap=$(dirname ${rp})

octave --persist --no-gui ${ap}/octave-script.m ${1}
