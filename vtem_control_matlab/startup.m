% parameters
vtemDeviceAddress = "192.168.4.3";
vtemPort = 502;

% set RMW Implementation
setenv('RMW_IMPLEMENTATION','')

% load custom messages
pkgFolderPath = fullfile(pwd, "../");
ros2genmsg(pkgFolderPath)