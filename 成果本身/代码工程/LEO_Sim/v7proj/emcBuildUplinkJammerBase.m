function pJsum_base_mW_UL = emcBuildUplinkJammerBase(visMask, pJsum_base_mW_DL, mode, fixed_dBm, reuseBias_dB)
%EMCBUILDUPLINKJAMMERBASE Build uplink jammer proxy power matrix.
%   Because the current V6 framework is downlink-centric, the uplink jammer
%   side is intentionally implemented as an engineering proxy that can be
%   replaced later by a more detailed satellite-receiver EMC model.

    pJsum_base_mW_UL = zeros(size(pJsum_base_mW_DL));
    mode = lower(strtrim(mode));

    switch mode
        case 'off'
            pJsum_base_mW_UL = zeros(size(pJsum_base_mW_DL));

        case 'fixed'
            p0 = 10.^(fixed_dBm/10);
            pJsum_base_mW_UL(visMask) = p0;

        otherwise  % reusedl
            pJsum_base_mW_UL = pJsum_base_mW_DL .* 10.^(reuseBias_dB/10);
            pJsum_base_mW_UL(~visMask) = 0;
    end
end
