function config = OpenSeqSLAMSetup()
    % Open the GUI, load the default config, and waiting until GUI is finished
    setupgui = SetupGUI();
    setupgui.loadConfigFromXML(fullfile(toolboxRoot(), '.config', 'default.xml'));
    uiwait(setupgui.hFig);

    % Save the returned parameters if the ui safely completed
    if setupgui.done
        config = setupgui.config;
    else
        config = [];
    end
end
