function [results, config] = OpenSeqSLAM2Match(settings_xml, diff_m_file)
    results = []; config = [];

    % Add the toolbox to the path
    run(fullfile(fileparts(which('OpenSeqSLAM2')), 'tools', 'toolboxInit'));

    config = xml2settings(settings_xml);
    if isempty(config)
        error(['Loading configuration from ''' settings_xml ''' failed.']);
    end
    
    % Load difference matrix
    load(diff_m_file);
    
    diff_m = diff_m(1:config.reference.subsample_factor:end, 1:config.reference.subsample_factor:end);

    % Run the SeqSLAM process
    OpenSeqSLAMRunMatch(config, diff_m);

end
