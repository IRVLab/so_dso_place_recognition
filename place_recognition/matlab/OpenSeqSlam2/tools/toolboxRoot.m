function root = toolboxRoot()
    directories = strsplit(fileparts(which('toolboxRoot')), filesep());
    root = strjoin(directories(1:end-1), filesep());
end
