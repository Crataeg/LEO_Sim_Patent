function out = emcMergeStruct(defaultCfg, userCfg)
%EMCMERGESTRUCT Recursively merge userCfg into defaultCfg.
%   Missing fields fall back to defaultCfg.

    out = defaultCfg;
    if nargin < 2 || isempty(userCfg) || ~isstruct(userCfg)
        return;
    end

    f = fieldnames(userCfg);
    for i = 1:numel(f)
        key = f{i};
        val = userCfg.(key);
        if isfield(out, key) && isstruct(out.(key)) && isstruct(val)
            out.(key) = emcMergeStruct(out.(key), val);
        else
            out.(key) = val;
        end
    end
end
