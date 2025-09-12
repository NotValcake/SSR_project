function blkStruct = slblocks

% This function specifies that the library 'proj_lib'
% should appear in the Library Browser with the 
% name 'SSR project lib'

    Browser.Library = 'proj_lib';
    % 'mylib' is the name of the library

    Browser.Name = 'SSR project lib';
    % 'My Library' is the library name that appears
    % in the Library Browser

    blkStruct.Browser = Browser;