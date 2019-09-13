function str = consoleString(isprior, varargin)
    % If varargin is provided we expect to get the parameter name and value
    if ~isempty(varargin)
        batch = true;
        batchName = varargin{1};
        batchValue = varargin{2};
    else
        batch = false;
    end

    % Construct the string
    if isprior
        str = 'Running';
    else
        str = 'Finished';
    end
    str = [str ' OpenSeqSLAM job'];
    if batch
        str = [str sprintf(' (%s = %s)', batchName, num2str(batchValue))];
    end
    if isprior
        str = [str ':'];
    else
        str = [str '.'];
    end
end
