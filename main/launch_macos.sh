#!/bin/bash

programmi_python=(
    "controller.py"
    "action.py"
    "gesture.py"
    "camera.py"
    "detection.py"
    "position_updater.py"
    "sense.py"
    "perception.py"
)

directory_programmi="."

# Ciclo attraverso l'array dei programmi Python
for programma in "${programmi_python[@]}"; do
    osascript -e "tell application \"Terminal\" to do script \"cd $directory_programmi; python3 ./$programma; exec zsh\""
done
