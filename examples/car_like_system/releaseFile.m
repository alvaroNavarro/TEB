clc
ReleasePath = '/home/navarro/ThesisAlvaro-TEB/Navarro/build/bin/car_like_system';
MatlabPath = '/home/navarro/ThesisAlvaro-TEB/Navarro/Navarro/examples/car_like_system';

clear car_like_system_sfun

copyfile([ReleasePath '/car_like_system_sfun.mexa64'], [MatlabPath '/car_like_system_sfun.mexa64']);
delete([ReleasePath '/car_like_system_sfun.mexa64']);

disp('Finished..')