@echo off  
pip install pyinstaller pyserial  
pyinstaller --onefile --windowed --name af_panel --clean tools/af_panel.py  
echo Done: dist\af_panel.exe  