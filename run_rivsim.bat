@echo off
title RivSim - Autonomous Catamaran Digital Twin
cd /d "%~dp0"
echo Starting RivSim 2D Physics Simulator & SITL Server...
target\release\rivsim.exe --mode visual
pause
