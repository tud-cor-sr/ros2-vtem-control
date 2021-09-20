% set RMW Implementation
setenv('RMW_IMPLEMENTATION','')

% load custom messages
pkgFolderPath = fullfile(pwd, "..\");
ros2genmsg(pkgFolderPath)

% parameters
vtemDeviceAddress = "192.168.4.2";
vtemPort = 502;