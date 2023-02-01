rem
rem SDR Cube Flasher script

echo
echo OH2UG Flasher v1.00
echo

rem show options

dspicp -h


rem change com port number if different in your system

dspicp -i COM3 -b 115200 SDR-Cube.hex
