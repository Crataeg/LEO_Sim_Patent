function jamAgg = genJamAggFromG(netG, z, cCode, seqLen, numSteps)
    % Generate a jammer envelope sequence jamAgg(t) in [0,1] from InfoGAN G(z,c)
    zc = [z(:); cCode(:)];
    zc = dlarray(single(zc),'CB');

    y = predict(netG, zc);
    y = gather(extractdata(y));
    y = double(y(:))';

    x0 = linspace(1,numSteps,seqLen);
    jamAgg = interp1(x0, y, 1:numSteps, 'pchip', 'extrap');
    jamAgg = min(max(jamAgg,0),1);
    jamAgg = movmean(jamAgg, 9);
    jamAgg = jamAgg(:);
end
