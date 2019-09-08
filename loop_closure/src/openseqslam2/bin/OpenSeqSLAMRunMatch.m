function OpenSeqSLAMRunMatch(config, diff_m)
    % Console mode
    progress = ProgressConsoleMatch(config, diff_m);

    % Run with some cute strings surrounding
    fprintf(consoleString(true));
    progress.run();
    fprintf('%s\n', consoleString(false));
end
