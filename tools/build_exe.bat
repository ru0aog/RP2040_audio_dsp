@echo off
pip install pyinstaller pyserial sounddevice numpy pillow lameenc
pyinstaller --onefile --windowed --name af_panel --clean --collect-binaries sounddevice --hidden-import sounddevice --hidden-import numpy --hidden-import PIL tools/af_panel.py
echo Done: dist\af_panel.exe