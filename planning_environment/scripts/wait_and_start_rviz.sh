#!/bin/bash

vcg_name="planning_description_configuration_wizard.vcg"
sig_name="vcg_ready"

while [ ! -N "$sig_name" ]; do
    sleep .1
    echo waiting...
done

rm $sig_name
exec rosrun rviz rviz -d $vcg_name
