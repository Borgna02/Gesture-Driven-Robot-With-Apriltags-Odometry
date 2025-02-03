#!/bin/bash
export DISPLAY=:0
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# Imposta XDG_RUNTIME_DIR
export XDG_RUNTIME_DIR=/tmp/runtime-root
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

# Avvia Xvfb sul display :0
Xvfb :0 -screen 0 1024x768x16 &
sleep 5

# Avvia il window manager fluxbox
fluxbox &
sleep 2

# Avvia x11vnc per esportare il display :0
x11vnc -display :0 -nopw -forever &
sleep 2

# Avvia websockify (senza --index) per rendere il VNC accessibile via browser sulla porta 6080
/opt/websockify/run --web /opt/novnc --wrap-mode ignore 6080 localhost:5900 &
sleep 2

echo "Avvio CoppeliaSim con la scena PioneerScene..."
cd /opt/coppeliasim
./coppeliaSim.sh /opt/coppeliasim/PioneerScene.ttt
echo "CoppeliaSim terminato!"

tail -f /dev/null
