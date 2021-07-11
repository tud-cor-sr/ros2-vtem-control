% set RMW Implementation
setenv('RMW_IMPLEMENTATION','')

% load custom messages
pkgFolderPath = fullfile(pwd, "..\");
ros2genmsg(pkgFolderPath)