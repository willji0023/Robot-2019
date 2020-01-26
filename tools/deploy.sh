#!/bin/bash
HOST='10.35.12.2'

if [ $# -ne 1 ] ; then
    echo usage: ./deploy.sh DIRECTORY
    echo -e "DIRECTORY is the directory in which the robot code binary is located"
    exit 1
fi

# Stop old robot program and download new one
ssh lvuser@$HOST "rm -f /home/lvuser/frcUserProgram"
ssh lvuser@$HOST "killall -q netconsole-host || :"
scp $1/frcUserProgram lvuser@$HOST:/home/lvuser
ssh admin@$HOST "setcap 'cap_sys_nice=pe' /home/lvuser/frcUserProgram"

# Deploy .so files
# TODO

# Deploy contents of "deploy" directory
ssh lvuser@$HOST "mkdir -p /home/lvuser/deploy"
scp -r deploy/* lvuser@$HOST:/home/lvuser/deploy

# As far as I can tell, the order of this doesn't matter. It only comments out
# some stuff for the LabVIEW runtime that apparently isn't needed, and
# dramatically reduces memory usage.
# See https://github.com/wpilibsuite/EclipsePlugins/pull/154
ssh admin@$HOST "sed -i -e 's/^StartupDLLs/;StartupDLLs/' /etc/natinst/share/ni-rt.ini"

# Restart robot program
ssh lvuser@$HOST ". /etc/profile.d/natinst-path.sh; chmod a+x /home/lvuser/frcUserProgram; /usr/local/frc/bin/frcKillRobot.sh -t -r;"
