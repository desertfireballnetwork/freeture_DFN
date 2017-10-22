

## Configuration 
### Configuration file

Change device to 0.
Change exposure to acceptable parameters (depends on the camera).

### Extra
Some FITS headers need to be modified straight into the code: 
`src/Fits2D.cpp`
and
`src/Fits3D.cpp`

## Run

### List devices
`freeture -l`

### Configuration check
`freeture -c /opt/hdevillepoix-freeture/configuration.cfg  -m 1`

### Acquisition mode
`freeture -c /opt/hdevillepoix-freeture/configuration.cfg  -m 3`

